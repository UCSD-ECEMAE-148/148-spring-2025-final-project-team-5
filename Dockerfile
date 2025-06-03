# Multi-stage build for efficient image
FROM python:3.8-slim-buster as builder

# Install build dependencies including BLAS/LAPACK
RUN apt-get update && apt-get install -y \
    build-essential \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    libblas-dev \
    libatlas-base-dev \
    git \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for proper linking
ENV BLAS=/usr/lib/x86_64-linux-gnu/libopenblas.so
ENV LAPACK=/usr/lib/x86_64-linux-gnu/libopenblas.so

# Create virtual environment
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Upgrade pip and install build tools
RUN pip install --upgrade pip setuptools wheel cython

# Install numpy first with specific configuration
RUN pip install numpy==1.21.6 --no-binary numpy

# Install quadprog from source with proper flags
RUN apt-get update && apt-get install -y wget && \
    wget https://github.com/quadprog/quadprog/archive/v0.1.11.tar.gz && \
    tar -xzf v0.1.11.tar.gz && \
    cd quadprog-0.1.11 && \
    python setup.py build_ext --inplace && \
    python setup.py install && \
    cd .. && rm -rf quadprog-0.1.11 v0.1.11.tar.gz

# Install other scientific computing dependencies
RUN pip install \
    scipy==1.7.3 \
    matplotlib==3.5.3 \
    scikit-learn==1.0.2

# Install optimization libraries
RUN pip install \
    casadi==3.5.5 \
    cvxpy==1.2.3

# Install trajectory planning helpers
RUN pip install trajectory-planning-helpers==0.76

# Install additional utilities
RUN pip install \
    pandas==1.3.5 \
    pyyaml==6.0 \
    configparser==5.3.0 \
    tqdm==4.65.0

# Runtime stage
FROM python:3.8-slim-buster

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    libopenblas-base \
    libgomp1 \
    libgfortran5 \
    libquadmath0 \
    libatlas3-base \
    && rm -rf /var/lib/apt/lists/*

# Copy virtual environment from builder
COPY --from=builder /opt/venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Set library path
ENV LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# Set working directory
WORKDIR /app

# Create directory structure
RUN mkdir -p \
    /app/config \
    /app/inputs/tracks \
    /app/inputs/frictionmaps \
    /app/inputs/veh_dyn_info \
    /app/outputs \
    /app/logs

# Set Python path
ENV PYTHONPATH="/app:$PYTHONPATH"

# Default command
CMD ["python", "--version"]
