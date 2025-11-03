import pytest

@pytest.fixture(scope='session', autouse=True)
def setup_environment():
    import ament_cmake
    import rclpy
    import sensor_msgs
    import cv_bridge
    # Add any additional setup code here if necessary

    yield

    # Teardown code if needed
