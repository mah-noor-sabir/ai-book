#!/usr/bin/env python3
"""
Script to check Qdrant connection and run the export if available.
"""

import subprocess
import sys
import time
from pathlib import Path

def check_qdrant_running():
    """
    Check if Qdrant is running by trying to connect to the default port.
    """
    try:
        import requests
        response = requests.get("http://localhost:6333/health", timeout=5)
        return response.status_code == 200
    except:
        return False

def start_qdrant_docker():
    """
    Attempt to start Qdrant using Docker.
    """
    try:
        # Check if Docker is available
        result = subprocess.run(["docker", "--version"],
                              capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            print("Docker is not available. Please install Docker to run Qdrant.")
            return False

        # Check if Qdrant container is already running
        result = subprocess.run(["docker", "ps"],
                              capture_output=True, text=True, timeout=10)
        if "qdrant" in result.stdout:
            print("Qdrant container is already running.")
            return True

        # Stop any existing qdrant containers
        subprocess.run(["docker", "stop", "qdrant"],
                      capture_output=True, text=True)
        subprocess.run(["docker", "rm", "qdrant"],
                      capture_output=True, text=True)

        # Start Qdrant container
        print("Starting Qdrant with Docker...")
        subprocess.Popen([
            "docker", "run", "-d",
            "--name", "qdrant",
            "-p", "6333:6333",
            "-p", "6334:6334",
            "qdrant/qdrant:latest"
        ])

        # Wait a bit for the container to start
        time.sleep(10)
        return True
    except Exception as e:
        print(f"Could not start Qdrant with Docker: {e}")
        return False

def install_dependencies():
    """
    Install required dependencies.
    """
    import subprocess
    import sys

    print("Installing required dependencies...")

    # Install packages from requirements.txt
    result = subprocess.run([
        sys.executable, "-m", "pip", "install", "-r", "requirements.txt"
    ], capture_output=True, text=True)

    if result.returncode != 0:
        print(f"Error installing dependencies: {result.stderr}")
        return False

    # Install additional packages if not in requirements.txt
    packages = ["requests", "beautifulsoup4", "markdown"]
    for package in packages:
        try:
            __import__(package.replace("-", "_"))
        except ImportError:
            print(f"Installing {package}...")
            subprocess.run([sys.executable, "-m", "pip", "install", package],
                          capture_output=True, text=True)

    return True

def main():
    print("Checking environment for Qdrant export...")

    # Install dependencies
    if not install_dependencies():
        print("Failed to install dependencies.")
        sys.exit(1)

    # Check if Qdrant is running
    if not check_qdrant_running():
        print("Qdrant is not running on localhost:6333.")

        # Try to start Qdrant with Docker
        if start_qdrant_docker():
            print("Qdrant started successfully with Docker.")

            # Wait a bit more for Qdrant to be ready
            time.sleep(5)

            # Verify it's running now
            if not check_qdrant_running():
                print("Qdrant still not responding after startup. Please check Docker logs.")
                sys.exit(1)
        else:
            print("Could not start Qdrant automatically.")
            print("Please start Qdrant manually before running the export script.")
            print("You can start Qdrant with: docker run -p 6333:6333 qdrant/qdrant:latest")
            sys.exit(1)
    else:
        print("Qdrant is running on localhost:6333.")

    # Run the export script
    print("\nRunning the export script...")
    try:
        from export_to_qdrant import BookExporterToQdrant
        exporter = BookExporterToQdrant()
        exporter.export_to_qdrant()

        print("\nExport completed successfully!")

        # Test the search functionality
        print("\nTesting search functionality...")
        results = exporter.search_in_qdrant("embodied intelligence", limit=3)

        print(f"\nTop 3 results for 'embodied intelligence':")
        for i, result in enumerate(results, 1):
            content_preview = result.payload['content'][:200] + "..." if len(result.payload['content']) > 200 else result.payload['content']
            print(f"{i}. {result.payload['title']} ({result.payload['section']})")
            print(f"   Score: {result.score:.3f}")
            print(f"   Content: {content_preview}")
            print()

    except Exception as e:
        print(f"Error running export script: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()