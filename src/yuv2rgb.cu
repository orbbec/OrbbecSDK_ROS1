#include <cuda_runtime.h>
#include <stdio.h>
#include <cuda.h>
#include "orbbec_camera/yuv2rgb.h"

uint8_t *d_y = NULL, *d_u = NULL, *d_v = NULL, *d_rgb = NULL;

extern "C" void yuv422InitMemory(int width, int height) {
  cudaMalloc((void**)&d_y, width * height);
  cudaMalloc((void**)&d_u, width * height / 4);
  cudaMalloc((void**)&d_v, width * height / 4);
  cudaMalloc((void**)&d_rgb, width * height * 3);
}

extern "C" void yuv422FreeMemory() {
  cudaFree(d_y);
  cudaFree(d_u);
  cudaFree(d_v);
  cudaFree(d_rgb);
}

__global__ void yuv_to_rgb_kernel(uint8_t* y_plane, uint8_t* u_plane, uint8_t* v_plane,
                                  uint8_t* rgb, int width, int height) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < width && y < height) {
    float y_val = y_plane[y * width + x];
    float v_val = u_plane[(y / 2) * (width / 2) + (x / 2)] - 128;
    float u_val = v_plane[(y / 2) * (width / 2) + (x / 2)] - 128;

    float r = y_val + 1.402 * v_val;
    float g = y_val - 0.344 * u_val - 0.714 * v_val;
    float b = y_val + 1.772 * u_val;
    // printf("r: %f, g: %f, b: %f\n", r, g, b);

    rgb[(y * width + x) * 3 + 0] = r > 255.0 ? 255 : (r < 0.0 ? 0 : static_cast<uint8_t>(r));
    rgb[(y * width + x) * 3 + 1] = g > 255.0 ? 255 : (g < 0.0 ? 0 : static_cast<uint8_t>(g));
    rgb[(y * width + x) * 3 + 2] = b > 255.0 ? 255 : (b < 0.0 ? 0 : static_cast<uint8_t>(b));
  }
}

extern "C" void yuv422ToRgb(uint8_t* y_plane, uint8_t* u_plane, uint8_t* v_plane,
                            unsigned char* rgb, int width, int height) {
  dim3 block(32, 32);
  dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
  cudaMemcpy(d_y, y_plane, width * height, cudaMemcpyHostToDevice);
  cudaMemcpy(d_u, u_plane, width * height / 4, cudaMemcpyHostToDevice);
  cudaMemcpy(d_v, v_plane, width * height / 4, cudaMemcpyHostToDevice);
  yuv_to_rgb_kernel<<<grid, block>>>(d_y, d_u, d_v, d_rgb, width, height);
  // get cuda err
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    printf("CUDA error: %s\n", cudaGetErrorString(err));
  }
  cudaMemcpy(rgb, d_rgb, width * height * 3, cudaMemcpyDeviceToHost);

  cudaDeviceSynchronize();
}
