import pytest
from omni.isaac.lab.app import AppLauncher

@pytest.fixture(scope="session")
def isaac_sim():
    """Launch Omniverse once for the entire test session."""
    # Pass no extra arguments to prevent interference with pytest
    launcher = AppLauncher(headless=True, args=[])
    sim_app = launcher.app
    yield sim_app
    sim_app.close()