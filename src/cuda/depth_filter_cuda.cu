// =============================================================================
// CUDA Depth Filter Kernels — GPU-accelerated depth preprocessing
// =============================================================================
// Replaces CPU OpenCV operations with CUDA kernels for <1ms total processing.
// On Orin AGX: range filter ~0.1ms, temporal avg ~0.2ms, bilateral ~0.5ms
// =============================================================================

#include <cuda_runtime.h>
#include <cstdint>
#include <cstdio>
#include <cmath>

// ── CUDA error checking macro ──
#define CUDA_CHECK(call) \
  do { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
      fprintf(stderr, "CUDA error at %s:%d: %s\n", \
              __FILE__, __LINE__, cudaGetErrorString(err)); \
    } \
  } while(0)

// =============================================================================
// Kernel 1: Range Filter — set out-of-range and NaN pixels to NaN
// =============================================================================
__global__ void range_filter_kernel(
  float * depth,
  int width,
  int height,
  float min_depth,
  float max_depth)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) return;

  int idx = y * width + x;
  float val = depth[idx];

  // NaN check: NaN != NaN
  if (val != val || val < min_depth || val > max_depth) {
    depth[idx] = __int_as_float(0x7FC00000);  // NaN
  }
}

// =============================================================================
// Kernel 2: Temporal Averaging — weighted average of N depth frames
// =============================================================================
// Uses a ring buffer of depth frames stored in GPU memory
__global__ void temporal_average_kernel(
  const float * const * frame_buffer,  // Array of pointers to depth frames
  float * output,
  int width,
  int height,
  int num_frames)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) return;

  int idx = y * width + x;
  float sum = 0.0f;
  int valid_count = 0;

  for (int f = 0; f < num_frames; f++) {
    float val = frame_buffer[f][idx];
    // NaN check
    if (val == val) {
      sum += val;
      valid_count++;
    }
  }

  if (valid_count > 0) {
    output[idx] = sum / static_cast<float>(valid_count);
  } else {
    output[idx] = __int_as_float(0x7FC00000);  // NaN
  }
}

// =============================================================================
// Kernel 3: Bilateral Filter — edge-preserving smoothing on GPU
// =============================================================================
__global__ void bilateral_filter_kernel(
  const float * input,
  float * output,
  int width,
  int height,
  int radius,
  float sigma_color,
  float sigma_space)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) return;

  int idx = y * width + x;
  float center_val = input[idx];

  // If center is NaN, output NaN
  if (center_val != center_val) {
    output[idx] = __int_as_float(0x7FC00000);
    return;
  }

  float color_coeff = -0.5f / (sigma_color * sigma_color);
  float space_coeff = -0.5f / (sigma_space * sigma_space);

  float sum_weight = 0.0f;
  float sum_value = 0.0f;

  for (int dy = -radius; dy <= radius; dy++) {
    for (int dx = -radius; dx <= radius; dx++) {
      int nx = x + dx;
      int ny = y + dy;

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

      float neighbor_val = input[ny * width + nx];
      if (neighbor_val != neighbor_val) continue;  // Skip NaN

      float space_dist_sq = static_cast<float>(dx * dx + dy * dy);
      float color_dist_sq = (neighbor_val - center_val) * (neighbor_val - center_val);

      float weight = expf(space_dist_sq * space_coeff + color_dist_sq * color_coeff);

      sum_weight += weight;
      sum_value += weight * neighbor_val;
    }
  }

  output[idx] = (sum_weight > 0.0f) ? (sum_value / sum_weight) : center_val;
}

// =============================================================================
// Host API — called from C++ ROS node
// =============================================================================

extern "C" {

// Context for CUDA depth filter
struct CudaDepthFilterContext {
  float * d_depth_in;
  float * d_depth_out;
  float * d_temporal_frames[8];  // Max 8 frames in ring buffer
  float ** d_frame_ptrs;         // Device array of frame pointers
  int width;
  int height;
  int num_temporal_frames;
  int current_frame_idx;
  int frames_filled;
  cudaStream_t stream;
};

CudaDepthFilterContext * cuda_depth_filter_create(int width, int height, int temporal_window)
{
  auto * ctx = new CudaDepthFilterContext();
  ctx->width = width;
  ctx->height = height;
  ctx->num_temporal_frames = (temporal_window > 8) ? 8 : temporal_window;
  ctx->current_frame_idx = 0;
  ctx->frames_filled = 0;

  size_t frame_bytes = width * height * sizeof(float);

  CUDA_CHECK(cudaStreamCreate(&ctx->stream));
  CUDA_CHECK(cudaMalloc(&ctx->d_depth_in, frame_bytes));
  CUDA_CHECK(cudaMalloc(&ctx->d_depth_out, frame_bytes));

  for (int i = 0; i < ctx->num_temporal_frames; i++) {
    CUDA_CHECK(cudaMalloc(&ctx->d_temporal_frames[i], frame_bytes));
  }

  CUDA_CHECK(cudaMalloc(&ctx->d_frame_ptrs, ctx->num_temporal_frames * sizeof(float *)));

  return ctx;
}

void cuda_depth_filter_destroy(CudaDepthFilterContext * ctx)
{
  if (!ctx) return;

  cudaFree(ctx->d_depth_in);
  cudaFree(ctx->d_depth_out);
  for (int i = 0; i < ctx->num_temporal_frames; i++) {
    cudaFree(ctx->d_temporal_frames[i]);
  }
  cudaFree(ctx->d_frame_ptrs);
  cudaStreamDestroy(ctx->stream);
  delete ctx;
}

// Process a single depth frame through the full GPU pipeline
// Returns processing time in milliseconds
float cuda_depth_filter_process(
  CudaDepthFilterContext * ctx,
  float * h_depth,         // Host input depth (width * height floats)
  float * h_depth_out,     // Host output depth
  float min_depth,
  float max_depth,
  bool enable_temporal,
  bool enable_bilateral,
  int bilateral_radius,
  float bilateral_sigma_color,
  float bilateral_sigma_space)
{
  if (!ctx) return -1.0f;

  size_t frame_bytes = ctx->width * ctx->height * sizeof(float);

  cudaEvent_t start, stop;
  CUDA_CHECK(cudaEventCreate(&start));
  CUDA_CHECK(cudaEventCreate(&stop));
  CUDA_CHECK(cudaEventRecord(start, ctx->stream));

  // Upload depth to GPU
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_depth_in, h_depth, frame_bytes,
    cudaMemcpyHostToDevice, ctx->stream));

  // Block dimensions
  dim3 block(16, 16);
  dim3 grid(
    (ctx->width + block.x - 1) / block.x,
    (ctx->height + block.y - 1) / block.y);

  // ── Stage 1: Range filter ──
  range_filter_kernel<<<grid, block, 0, ctx->stream>>>(
    ctx->d_depth_in, ctx->width, ctx->height, min_depth, max_depth);

  // ── Stage 2: Temporal averaging ──
  float * current_output = ctx->d_depth_in;  // Default: no temporal

  if (enable_temporal && ctx->num_temporal_frames > 1) {
    // Copy current filtered frame to ring buffer
    CUDA_CHECK(cudaMemcpyAsync(
      ctx->d_temporal_frames[ctx->current_frame_idx],
      ctx->d_depth_in, frame_bytes,
      cudaMemcpyDeviceToDevice, ctx->stream));

    ctx->current_frame_idx = (ctx->current_frame_idx + 1) % ctx->num_temporal_frames;
    if (ctx->frames_filled < ctx->num_temporal_frames) {
      ctx->frames_filled++;
    }

    if (ctx->frames_filled > 1) {
      // Upload frame pointers
      CUDA_CHECK(cudaMemcpyAsync(ctx->d_frame_ptrs, ctx->d_temporal_frames,
        ctx->frames_filled * sizeof(float *),
        cudaMemcpyHostToDevice, ctx->stream));

      temporal_average_kernel<<<grid, block, 0, ctx->stream>>>(
        (const float * const *)ctx->d_frame_ptrs,
        ctx->d_depth_out,
        ctx->width, ctx->height, ctx->frames_filled);

      current_output = ctx->d_depth_out;
    }
  }

  // ── Stage 3: Bilateral filter ──
  if (enable_bilateral) {
    float * bilateral_in = current_output;
    float * bilateral_out = (current_output == ctx->d_depth_out) ?
      ctx->d_depth_in : ctx->d_depth_out;

    bilateral_filter_kernel<<<grid, block, 0, ctx->stream>>>(
      bilateral_in, bilateral_out,
      ctx->width, ctx->height,
      bilateral_radius, bilateral_sigma_color, bilateral_sigma_space);

    current_output = bilateral_out;
  }

  // Download result
  CUDA_CHECK(cudaMemcpyAsync(h_depth_out, current_output, frame_bytes,
    cudaMemcpyDeviceToHost, ctx->stream));

  CUDA_CHECK(cudaEventRecord(stop, ctx->stream));
  CUDA_CHECK(cudaStreamSynchronize(ctx->stream));

  float elapsed_ms = 0.0f;
  CUDA_CHECK(cudaEventElapsedTime(&elapsed_ms, start, stop));

  CUDA_CHECK(cudaEventDestroy(start));
  CUDA_CHECK(cudaEventDestroy(stop));

  return elapsed_ms;
}

}  // extern "C"
