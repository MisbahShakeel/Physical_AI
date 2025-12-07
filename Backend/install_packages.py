"""
Installation script for Textbook RAG System dependencies
"""
import subprocess
import sys
import os
from pathlib import Path

def install_packages():
    """Install all required packages from requirements.txt"""
    print("Installing Textbook RAG System dependencies...")

    # Change to the Backend directory
    backend_dir = Path(__file__).parent
    os.chdir(backend_dir)

    try:
        # Upgrade pip first
        print("Upgrading pip...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])

        # Install packages from requirements.txt
        print("Installing packages from requirements.txt...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])

        print("\n✅ All packages installed successfully!")
        print(f"Installation location: {backend_dir}")

        # Verify installations
        print("\n🔍 Verifying key package installations...")
        verify_installations()

    except subprocess.CalledProcessError as e:
        print(f"❌ Error during installation: {e}")
        return False
    except FileNotFoundError:
        print("❌ requirements.txt file not found!")
        return False

    return True

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
            print(f"✅ {package_name} - OK")
        except ImportError:
            print(f"❌ {package_name} - FAILED")

    print("\n🔍 Checking for potential conflicts...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "check"])
        print("✅ No package conflicts found")
    except subprocess.CalledProcessError:
        print("⚠️  Package conflicts detected - this may cause issues")

def create_virtual_environment():
    """Create a virtual environment for the project (optional)"""
    print("\nWould you like to create a virtual environment? (y/n): ", end="")
    response = input().lower().strip()

    if response in ['y', 'yes']:
        try:
            # Create virtual environment
            venv_dir = Path(__file__).parent / "venv"
            print(f"Creating virtual environment at {venv_dir}...")

            subprocess.check_call([sys.executable, "-m", "venv", str(venv_dir)])

            # Install packages in virtual environment
            pip_path = venv_dir / "Scripts" / "pip.exe" if os.name == 'nt' else venv_dir / "bin" / "pip"
            subprocess.check_call([str(pip_path), "install", "--upgrade", "pip"])
            subprocess.check_call([str(pip_path), "install", "-r", "requirements.txt"])

            print(f"✅ Virtual environment created successfully at {venv_dir}")
            print("To activate the virtual environment, run:")
            if os.name == 'nt':
                print(f"    {venv_dir / 'Scripts' / 'activate.bat'}")
            else:
                print(f"    source {venv_dir / 'bin' / 'activate'}")

        except subprocess.CalledProcessError as e:
            print(f"❌ Error creating virtual environment: {e}")
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