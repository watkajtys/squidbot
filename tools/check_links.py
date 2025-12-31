import os
import re
import urllib.parse
from pathlib import Path

def find_markdown_files(root_dir):
    md_files = []
    for root, dirs, files in os.walk(root_dir):
        # Skip .git and node_modules if present
        if '.git' in dirs:
            dirs.remove('.git')
        if 'node_modules' in dirs:
            dirs.remove('node_modules')
            
        for file in files:
            if file.endswith('.md'):
                md_files.append(Path(root) / file)
    return md_files

def check_links(files):
    broken_links = []
    # Regex to match markdown links: [text](url)
    link_pattern = re.compile(r'[[^]]+]\[([^)]+)\]')
    
    for file_path in files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            continue
            
        matches = link_pattern.findall(content)
        for text, url in matches:
            # Skip external links and anchors
            if url.startswith(('http://', 'https://', 'mailto:', '#')):
                continue
            
            # Remove anchor from url if present
            clean_url = url.split('#')[0]
            if not clean_url:
                continue

            # Handle absolute paths (relative to root? usually not in standard md but possible)
            # Assuming relative paths for now
            
            # Construct absolute path
            target_path = (file_path.parent / clean_url).resolve()
            
            # Check if file exists
            if not target_path.exists():
                # Try decoding URL encoding (e.g. %20 -> space)
                decoded_url = urllib.parse.unquote(clean_url)
                target_path_decoded = (file_path.parent / decoded_url).resolve()
                
                if not target_path_decoded.exists():
                     # Check if it is a directory
                    if target_path_decoded.parent.exists() and target_path_decoded.suffix == '':
                         # Maybe it's a directory link?
                         pass
                    else:
                        broken_links.append({
                            'source': file_path,
                            'link_text': text,
                            'target_url': url,
                            'resolved_path': target_path
                        })

    return broken_links

def main():
    root_dir = '.'
    print(f"Scanning for markdown files in {root_dir}...")
    files = find_markdown_files(root_dir)
    print(f"Found {len(files)} markdown files.")
    
    print("Checking links...")
    broken_links = check_links(files)
    
    if broken_links:
        print(f"\nFound {len(broken_links)} broken links:")
        for link in broken_links:
            print(f"File: {link['source']}")
            print(f"  Link: [{link['link_text']}]({link['target_url']})")
            print(f"  Resolved: {link['resolved_path']}")
            print("-" * 20)
    else:
        print("\nNo broken links found!")

if __name__ == "__main__":
    main()
