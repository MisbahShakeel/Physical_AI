"""
Flexible installation script for Textbook RAG System dependencies
"""
import subprocess
import sys
import os
from pathlib import Path

def install_packages():
    """Install all required packages from flexible requirements.txt"""
    print("[PACKAGE] Installing Textbook RAG System dependencies (Flexible versions)...")

    # Change to the Backend directory
    backend_dir = Path(__file__).parent
    os.chdir(backend_dir)

    try:
        # Upgrade pip first
        print("[PIP] Upgrading pip...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])

        # Install packages from flexible requirements.txt
        print("[REQ] Installing packages from requirements_flexible.txt...")

        # Try to install with the flexible requirements
        result = subprocess.run([
            sys.executable, "-m", "pip", "install", "-r", "requirements_flexible.txt", "--timeout", "300"
        ], capture_output=True, text=True)

        if result.returncode != 0:
            print(f"[WARN] Some packages failed to install. Error: {result.stderr}")
            print("[INFO] Installing core packages individually...")

            # Install core packages one by one
            core_packages = [
                "fastapi",
                "uvicorn[standard]",
                "openai",
                "qdrant-client",
                "asyncpg",
                "pydantic",
                "pydantic-settings",
                "python-dotenv",
                "cryptography",
                "redis",
                "numpy",
                "scikit-learn",
                "pandas",
                "aiohttp",
                "httpx",
                "pyjwt",
                "langchain",
                "pytest",
                "pytest-asyncio",
                "slowapi",
                "aioredis"
            ]

            failed_packages = []
            for package in core_packages:
                print(f"[REQ] Installing {package}...")
                try:
                    subprocess.check_call([sys.executable, "-m", "pip", "install", package])
                    print(f"[OK] Successfully installed {package}")
                except subprocess.CalledProcessError as e:
                    print(f"[ERROR] Failed to install {package}: {e}")
                    failed_packages.append(package)

            if failed_packages:
                print(f"[WARN] Failed to install packages: {failed_packages}")
            else:
                print("[SUCCESS] All core packages installed successfully!")
        else:
            print("[SUCCESS] All packages from requirements installed successfully!")

        print(f"Installation location: {backend_dir}")

        # Verify installations
        print("\n[VERIFY] Verifying key package installations...")
        verify_installations()

        return True

    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Error during installation: {e}")
        return False
    except FileNotFoundError:
        print("[ERROR] requirements_flexible.txt file not found!")
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

            # Install core packages in the virtual environment
            core_packages = [
                "fastapi",
                "uvicorn[standard]",
                "openai",
                "qdrant-client",
                "asyncpg",
                "pydantic",
                "pydantic-settings",
                "python-dotenv",
                "cryptography",
                "redis",
                "numpy",
                "scikit-learn",
                "aiohttp",
                "httpx",
                "pyjwt",
                "langchain",
                "pytest",
                "slowapi",
                "aioredis"
            ]

            for package in core_packages:
                try:
                    subprocess.check_call([str(pip_path), "install", package])
                    print(f"[OK] Installed {package} in virtual environment")
                except subprocess.CalledProcessError:
                    print(f"[WARN] Could not install {package} in virtual environment")

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
    print("[PACKAGE] Textbook RAG System - Flexible Package Installation")
    print("=" * 60)

    # Install packages
    success = install_packages()

    if success:
        # Ask about virtual environment
        create_virtual_environment()

        print("\n" + "=" * 60)
        print("[SUCCESS] Installation completed!")
        print("\nNext steps:")
        print("1. Set up your environment variables (API keys, database URLs, etc.)")
        print("2. Configure your Qdrant and PostgreSQL connections")
        print("3. Run the application with: uvicorn rag_system:app --reload")
        print("4. Run tests with: pytest")
    else:
        print("\n[ERROR] Installation failed. Please check the error messages above.")
        sys.exit(1)