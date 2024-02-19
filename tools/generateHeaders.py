import os

def generate_header_file(source_file):
    if not source_file.endswith(".cpp"):
        print("Error: Input file must have a .cpp extension.")
        return

    header_file = source_file[:-4] + ".hpp"
    
    if os.path.exists(header_file):
        print("Error: Header file already exists.")
        return

    with open(header_file, "w") as header:
        header.write("#pragma once\n\n")
        
        with open(source_file, "r") as source:
            for line in source:
                if line.startswith("#include"):
                    header.write(line)
                elif line.startswith("class"):
                    header.write(line.rstrip() + ";\n")
                elif "(" in line and ")" in line and ";" in line:
                    function_declaration = line.split(")")[1].split(";")[0].strip() + ";\n"
                    header.write(function_declaration)
                elif line.startswith("int main()"):
                    break

    print(f"Header file {header_file} generated successfully.")

if __name__ == "__main__":
    source_file = input("Enter the name of the source file (.cpp): ")
    generate_header_file(source_file)