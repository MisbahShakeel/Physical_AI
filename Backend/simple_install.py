"""
Simple installation script for Textbook RAG System dependencies
"""
import subprocess
import sys
import os
from pathlib import Path

def install_packages():
    """Install all required packages from requirements.txt"""
    print("[PACKAGE] Installing Textbook RAG System dependencies...")

    # Change to the Backend directory
    backend_dir = Path(__file__).parent
    os.chdir(backend_dir)

    try:
        # Upgrade pip first
        print("[PIP] Upgrading pip...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])

        # Install packages from requirements.txt, but handle psycopg2-binary separately
        print("[REQ] Installing packages from requirements.txt...")

        # Read requirements and install in phases to handle problematic packages
        with open("requirements.txt", "r") as f:
            lines = f.readlines()

        # Separate psycopg2-binary since it causes issues
        core_packages = []
        special_packages = []

        for line in lines:
            line = line.strip()
            if line and not line.startswith("#"):
                if "psycopg2-binary" in line:
                    special_packages.append(line)
                else:
                    core_packages.append(line)

        # Write core packages to a temporary file
        with open("core_requirements.txt", "w") as f:
            for pkg in core_packages:
                f.write(pkg + "\n")

        # Install core packages first
        print("[REQ] Installing core packages...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "core_requirements.txt"])

        # Try to install psycopg2-binary separately (may fail on some systems)
        if special_packages:
            print("[REQ] Installing database packages (may take longer)...")
            for pkg in special_packages:
                try:
                    subprocess.check_call([sys.executable, "-m", "pip", "install", pkg])
                    print(f"[OK] Successfully installed {pkg}")
                except subprocess.CalledProcessError:
                    print(f"[WARN] Failed to install {pkg}, using alternatives...")
                    # Try to install asyncpg as an alternative
                    try:
                        subprocess.check_call([sys.executable, "-m", "pip", "install", "asyncpg"])
                        print("[OK] Installed asyncpg as database alternative")
                    except:
                        print("[WARN] Could not install database packages")

        print("\n[SUCCESS] Most packages installed successfully!")
        print(f"Installation location: {backend_dir}")

        # Verify installations
        print("\n[VERIFY] Verifying key package installations...")
        verify_installations()

        # Clean up temp file
        if os.path.exists("core_requirements.txt"):
            os.remove("core_requirements.txt")

        return True

    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Error during installation: {e}")
        return False
    except FileNotFoundError:
        print("[ERROR] requirements.txt file not found!")
        return False

def verify_installations():
    """Verify that key packages are properly installed"""
    packages_to_verify = [
        ("fastapi", "fastapi"),
        ("uvicorn", "uvicorn"),
        ("openai", "openai"),
        ("qdrant_client", "qdrant_client"),
        ("asyncpg", "asyncpg"),
        ("pydantic", "pydantic"),
        ("numpy", "numpy"),
        ("pytest", "pytest"),
        ("redis", "redis"),
        ("slowapi", "slowapi"),
        ("langchain", "langchain"),
        ("cryptography", "cryptography"),
    ]

    for package_name, import_name in packages_to_verify:
        try:
            __import__(import_name)
            print(f"[OK] {package_name} - Available")
        except ImportError:
            print(f"[MISSING] {package_name} - Not available")

    print("\n[CHECK] Checking for potential conflicts...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "check"])
        print("[OK] No package conflicts found")
    except subprocess.CalledProcessError:
        print("[WARN] Package conflicts detected - this may cause issues")

def create_virtual_environment():
    """Create a virtual environment for the project (optional)"""
    print("\n[ENV] Would you like to create a virtual environment? (y/n): ", end="")
    try:
        response = input().lower().strip()
    except EOFError:
        response = 'n'  # Default to no if input fails

    if response in ['y', 'yes']:
        try:
            # Create virtual environment
            venv_dir = Path(__file__).parent / "venv"
            print(f"[ENV] Creating virtual environment at {venv_dir}...")

            subprocess.check_call([sys.executable, "-m", "venv", str(venv_dir)])

            # Install packages in virtual environment
            if os.name == 'nt':  # Windows
                pip_path = venv_dir / "Scripts" / "pip.exe"
            else:  # Unix/Linux/Mac
                pip_path = venv_dir / "bin" / "pip"

            subprocess.check_call([str(pip_path), "install", "--upgrade", "pip"])

            # Copy the installation logic to the virtual environment
            # For now, just install the core requirements
            subprocess.check_call([str(pip_path), "install", "-r", "requirements.txt"])

            print(f"[SUCCESS] Virtual environment created successfully at {venv_dir}")
            print("To activate the virtual environment, run:")
            if os.name == 'nt':
                print(f"    {venv_dir / 'Scripts' / 'activate.bat'}")
            else:
                print(f"    source {venv_dir / 'bin' / 'activate'}")

        except subprocess.CalledProcessError as e:
            print(f"[ERROR] Error creating virtual environment: {e}")
            return False

    return True

if __name__ == "__main__":
    print("[PACKAGE] Textbook RAG System - Package Installation")
    print("=" * 50)

    # Install packages
    success = install_packages()

    if success:
        # Ask about virtual environment
        create_virtual_environment()

        print("\n" + "=" * 50)
        print("[SUCCESS] Installation completed!")
        print("\nNext steps:")
        print("1. Set up your environment variables (API keys, database URLs, etc.)")
        print("2. Configure your Qdrant and PostgreSQL connections")
        print("3. Run the application with: uvicorn rag_system:app --reload")
        print("4. Run tests with: pytest")
    else:
        print("\n[ERROR] Installation failed. Please check the error messages above.")
        sys.exit(1)