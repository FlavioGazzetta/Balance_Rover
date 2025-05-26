#!/usr/bin/env python3
import subprocess
import sys
import os
import shutil

# If your compiler lives outside the venv, prepend its bin folder here:
EXTRA_PATHS = [
    r"C:\msys64\mingw64\bin", 
    r"C:\MinGW\bin",
    r"C:\TDM-GCC-64\bin"
]
# Update PATH so shutil.which can see g++, clang++, etc.
os.environ["PATH"] = ";".join(EXTRA_PATHS) + ";" + os.environ.get("PATH","")

def find_compiler():
    # 1) Respect CXX env var
    env_comp = os.getenv("CXX")
    if env_comp and shutil.which(env_comp):
        return shutil.which(env_comp)
    # 2) Try common names
    for name in ("g++", "clang++", "c++", "cl"):
        path = shutil.which(name)
        if path:
            return path
    return None

def main():
    if len(sys.argv) != 2:
        print("Usage: python run_cpp.py <cpp_file>")
        sys.exit(1)

    cpp = sys.argv[1]
    if not os.path.isfile(cpp):
        print(f"Error: '{cpp}' not found.")
        sys.exit(1)

    compiler = find_compiler()
    if not compiler:
        print("Error: no C++ compiler found.")
        sys.exit(1)

    # Defaults—override via env vars if needed
    muj_inc = os.getenv("MUJOCO_INCLUDE", r"C:/mujoco/mujoco210/include")
    muj_lib = os.getenv("MUJOCO_LIB",     r"C:/mujoco/mujoco210/bin")
    glf_inc = os.getenv("GLFW_INCLUDE",   r"C:/glfw-3.3.8-mingw-w64/include")
    glf_lib = os.getenv("GLFW_LIB",       r"C:/glfw-3.3.8-mingw-w64/lib-mingw-w64")

    base, _ = os.path.splitext(cpp)
    exe = base + (".exe" if sys.platform.startswith("win") else "")

    # MSVC vs. GCC/Clang flags
    if os.path.basename(compiler).lower().startswith("cl"):
        cmd = [
            compiler, cpp,
            f'/I{muj_inc}', f'/I{glf_inc}', '/EHsc',
            '/link',
            f'/LIBPATH:{muj_lib}', 'mujoco210.lib',
            f'/LIBPATH:{glf_lib}', 'glfw3.lib',
            f'/OUT:{exe}'
        ]
    else:
        cmd = [
            compiler, cpp,
            "-std=c++17", "-O2",
            f"-I{muj_inc}", f"-I{glf_inc}",
            f"-L{muj_lib}", f"-L{glf_lib}",
            "-lmujoco210", "-lglfw3", "-lgdi32",
            "-o", exe
        ]

    print("Compiling with:", compiler)
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode:
        print("❌ Compile error:\n", result.stderr)
        sys.exit(1)

    print("✅ Build succeeded. Running…")
    run = subprocess.run([exe], capture_output=True, text=True)
    if run.returncode:
        print("❌ Runtime error:\n", run.stderr)
        sys.exit(1)

    print("📤 Program output:")
    print(run.stdout)

if __name__ == "__main__":
    main()
