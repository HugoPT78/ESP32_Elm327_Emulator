import os

def combine_arduino_files(output_filename="fullcode.md"):
    # Define the file extensions to include
    target_extensions = {'.ino', '.h', '.cpp', '.json'}
    
    # Get the name of the current script to exclude it
    current_script = os.path.basename(__file__)
    
    with open(output_filename, 'w', encoding='utf-8') as outfile:
        # Iterate over all files in the current directory
        for filename in os.listdir('.'):
            # Skip directories and the current script
            if os.path.isfile(filename) and filename != current_script:
                # Check if the file has one of the target extensions
                _, ext = os.path.splitext(filename)
                if ext.lower() in target_extensions:
                    # Write the filename as a heading in Markdown
                    outfile.write(f"# {filename}\n\n")
                    
                    # Write the file content
                    with open(filename, 'r', encoding='utf-8') as infile:
                        outfile.write(infile.read())
                    
                    # Add a separator between files
                    outfile.write("\n\n---\n\n")

if __name__ == "__main__":
    combine_arduino_files()
