# git clone https://github.com/stephane/libmodbus

using Clang.Generators
using Clang.LibClang.Clang_jll

cd(@__DIR__)

clang_dir = "nidaqmxc/"
# wrapper generator options
options = load_options(joinpath(@__DIR__, "generator.toml"))

# add compiler flags, e.g. "-DXXXXXXXXX"
args = get_default_args()
push!(args, "-I$clang_dir")

headers = [joinpath(clang_dir, header) for header in readdir(clang_dir) if endswith(header, ".h")]
# there is also an experimental `detect_headers` function for auto-detecting top-level headers in the directory
# headers = detect_headers(clang_dir, args)

# create context
ctx = create_context(headers, args, options)

# run generator
build!(ctx)

# add to the file the following line after module: 
# const libmodbus = "libmodbus"