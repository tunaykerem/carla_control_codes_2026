#include "carla_reader_clang/zed_camera_reader.hpp"
#include "carla_reader_clang/tcp_sender.hpp"
#include "profiling.hpp"
#include <carla/client/BlueprintLibrary.h>
#include <carla/image/ImageView.h>
#include <iostream>
#include <cstring>
#include <algorithm>

// libjpeg for JPEG compression (backed by libjpeg-turbo8 for speed)
#include <jpeglib.h>

namespace carla_sensor_bridge {

// ── JPEG memory-destination manager ─────────────────────────────────────────
// Writes JPEG output to a std::vector<uint8_t> instead of a file.
namespace {

struct VectorDest {
    struct jpeg_destination_mgr pub;
    std::vector<uint8_t>* buffer;
    static constexpr size_t BLOCK_SIZE = 65536;
};

static void init_destination(j_compress_ptr cinfo) {
    auto* dest = reinterpret_cast<VectorDest*>(cinfo->dest);
    dest->buffer->resize(VectorDest::BLOCK_SIZE);
    dest->pub.next_output_byte = dest->buffer->data();
    dest->pub.free_in_buffer = dest->buffer->size();
}

static boolean empty_output_buffer(j_compress_ptr cinfo) {
    auto* dest = reinterpret_cast<VectorDest*>(cinfo->dest);
    size_t old_size = dest->buffer->size();
    dest->buffer->resize(old_size + VectorDest::BLOCK_SIZE);
    dest->pub.next_output_byte = dest->buffer->data() + old_size;
    dest->pub.free_in_buffer = VectorDest::BLOCK_SIZE;
    return TRUE;
}

static void term_destination(j_compress_ptr cinfo) {
    auto* dest = reinterpret_cast<VectorDest*>(cinfo->dest);
    size_t used = dest->buffer->size() - dest->pub.free_in_buffer;
    dest->buffer->resize(used);
}

} // anonymous namespace


ZedCameraReader::ZedCameraReader(carla::SharedPtr<carla::client::Actor> parent,
                                 carla::client::World& world) {
    auto bp_lib = world.GetBlueprintLibrary();
    auto camera_bp = *(bp_lib->Find("sensor.camera.rgb"));

    camera_bp.SetAttribute("image_size_x", "1280");
    camera_bp.SetAttribute("image_size_y", "720");
    camera_bp.SetAttribute("fov", "90.0");
    camera_bp.SetAttribute("sensor_tick", "0.05");              // 20 Hz
    camera_bp.SetAttribute("enable_postprocess_effects", "true");

    // URDF transform for zed_left: (0.85, -0.06, 1.00, 0, 0, 0)
    carla::geom::Transform transform(
        carla::geom::Location(0.85f, -0.06f, 1.00f),
        carla::geom::Rotation(0.0f, 0.0f, 0.0f)
    );

    auto actor = world.SpawnActor(camera_bp, transform, parent.get());
    sensor_ = std::static_pointer_cast<carla::client::Sensor>(actor);

    if (sensor_) {
        sensor_->Listen([this](auto data) { this->onImageData(data); });
        std::cout << "[CARLA Reader] Spawned ZED Left Camera (ID: "
                  << sensor_->GetId() << ") [JPEG Q" << JPEG_QUALITY << "]\n";
    } else {
        std::cerr << "[CARLA Reader] ERROR: Failed to cast Camera Actor to Sensor!\n";
    }
}

ZedCameraReader::~ZedCameraReader() {
    destroy();
}

void ZedCameraReader::destroy() {
    if (sensor_) {
        try {
            sensor_->Stop();
            sensor_->Destroy();
        } catch (...) {}
        sensor_ = nullptr;
    }
}

void ZedCameraReader::onImageData(carla::SharedPtr<carla::sensor::SensorData> data) {
    PROF_READ_BEGIN("ZED");

    auto image = std::dynamic_pointer_cast<carla::sensor::data::Image>(data);
    if (!image) return;

    uint32_t width  = image->GetWidth();
    uint32_t height = image->GetHeight();
    if (width == 0 || height == 0) return;

    double timestamp = image->GetTimestamp();

    // CARLA image: BGRA, 4 bytes per pixel
    const auto* raw_pixels = reinterpret_cast<const uint8_t*>(image->data());
    size_t num_pixels = static_cast<size_t>(width) * height;

    // ── 1. BGRA → RGB with brightness ×3.5 (single pass) ────────────────
    //    libjpeg expects RGB input, so we convert BGR→RGB here too.
    bgr_buffer_.resize(num_pixels * 3);
    for (size_t i = 0; i < num_pixels; i++) {
        // BGRA layout: [B, G, R, A] → output as RGB for libjpeg
        bgr_buffer_[i * 3 + 0] = static_cast<uint8_t>(std::min(255, static_cast<int>(raw_pixels[i * 4 + 2] * BRIGHTNESS_SCALE))); // R
        bgr_buffer_[i * 3 + 1] = static_cast<uint8_t>(std::min(255, static_cast<int>(raw_pixels[i * 4 + 1] * BRIGHTNESS_SCALE))); // G
        bgr_buffer_[i * 3 + 2] = static_cast<uint8_t>(std::min(255, static_cast<int>(raw_pixels[i * 4 + 0] * BRIGHTNESS_SCALE))); // B
    }

    PROF_READ_T1();

    // ── 2. JPEG compress with libjpeg ────────────────────────────────────
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    // Set up memory destination
    VectorDest dest;
    dest.buffer = &jpeg_buffer_;
    dest.pub.init_destination = init_destination;
    dest.pub.empty_output_buffer = empty_output_buffer;
    dest.pub.term_destination = term_destination;
    cinfo.dest = reinterpret_cast<struct jpeg_destination_mgr*>(&dest);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    // Write scanlines
    int row_stride = static_cast<int>(width) * 3;
    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row_pointer = bgr_buffer_.data() + cinfo.next_scanline * row_stride;
        jpeg_write_scanlines(&cinfo, &row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    // ── 3. Build payload: [width(4B) | height(4B) | JPEG data] ──────────
    size_t jpeg_size = jpeg_buffer_.size();
    size_t total_payload = sizeof(uint32_t) * 2 + jpeg_size;
    send_buffer_.resize(total_payload);
    std::memcpy(send_buffer_.data(),     &width,  sizeof(uint32_t));
    std::memcpy(send_buffer_.data() + 4, &height, sizeof(uint32_t));
    std::memcpy(send_buffer_.data() + 8, jpeg_buffer_.data(), jpeg_size);

    // ── 4. Send via TCP Bridge ───────────────────────────────────────────
    TCPSender::getInstance().send(SensorType::ZED_LEFT_IMAGE, timestamp,
                                  send_buffer_.data(), static_cast<uint32_t>(total_payload));

    PROF_READ_END("ZED");
}

} // namespace carla_sensor_bridge
