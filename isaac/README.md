 ## **Minimum System Requirements:**

**Operating System:**
- Windows (this is a .bat script)
- 64-bit required

**Graphics Card (Critical):**
- **NVIDIA GPU with RTX series recommended** (RTX 2060 or better)
- Must support CUDA 11.8 or 12.1
- **Minimum 8GB VRAM**, preferably 12GB+ for complex simulations
- Driver version 552.86+ required for CUDA 12

**CPU:**
- Intel i7 or AMD Ryzen 7 (8+ cores recommended)
- Isaac Sim is CPU-intensive for physics calculations

**Memory:**
- **Minimum 32GB RAM** (64GB recommended for large simulations)
- Isaac Sim loads entire scenes into memory

**Storage:**
- **50-100GB free space** minimum
- SSD strongly recommended (Isaac Sim has large asset files)
- The script mentions "significant disk space" requirement

**Software Prerequisites:**
- Anaconda or Miniconda
- Git
- NVIDIA CUDA drivers
- Python 3.10 (handled by the script)

## **Why It's Demanding:**

1. **Isaac Sim** is NVIDIA's professional robotics simulator - it's enterprise-grade software
2. **CUDA requirement** means you need an NVIDIA GPU (no AMD or integrated graphics)
3. **Large memory footprint** due to physics simulation and 3D rendering
4. **Real-time ray tracing** capabilities for photorealistic simulation

## **What Won't Work:**
- Laptops with integrated graphics
- AMD graphics cards
- Older NVIDIA cards (GTX series might struggle)
- Systems with less than 16GB RAM
- Computers without CUDA support 