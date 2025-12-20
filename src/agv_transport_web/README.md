# AGV Transport Web Dashboard

A web-based dashboard for monitoring and controlling the AGV transport system.

## Overview

This package provides a FastAPI-based web interface that bridges HTTP requests with the ROS2 transport system. It offers real-time monitoring of AGV status, task queue management, and a simple interface for creating transport orders.

## Features

- Real-time AGV status monitoring
- Task queue visualization
- Transport order creation interface
- Integration with existing ROS2 transport infrastructure

## Dependencies

- FastAPI for web server
- ROS2 (Jazzy) for system integration
- Jinja2 for HTML templating
- Pydantic for data validation

## Installation

This package is designed to be built as part of the AGV_2 workspace:

```bash
cd /ros2_ws
colcon build --packages-select agv_transport_web
source install/setup.bash
```

## Usage

The web dashboard will be launched as part of the complete AGV system. Detailed usage instructions will be provided in subsequent implementation phases.

## Development

For development, install the Python dependencies:

```bash
pip install -r requirements.txt
```

Run tests:

```bash
pytest
```

## License

MIT License