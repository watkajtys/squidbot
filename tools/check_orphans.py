import os
import re
from pathlib import Path

def normalize_path(path):
    return str(Path(path).resolve()).lower()

def find_markdown_files(root_dir):
    md_files = set()
    print(f"DEBUG: Scanning {root_dir}")
    for root, dirs, files in os.walk(root_dir):
        if '.git' in dirs: dirs.remove('.git')
        if 'node_modules' in dirs: dirs.remove('node_modules')
        
        for file in files:
            if file.endswith('.md'):
                full_path = normalize_path(os.path.join(root, file))
                md_files.add(full_path)
    return md_files

def get_all_links(files):
    linked_files = set()
    
    # Regex for [text](url)
    link_pattern = re.compile(r'\[([^\]]+)\]\(([^)]+)\)')
    
    for file_path in files:
        # We need the original case for file_path to open it, 
        # but we stored lower() in the set.
        # Since we don't have the map back, we will re-walk or just try to open.
        # Actually, let's just pass the set of absolute paths (which we have) 
        # but we need to know the 'real' path to open it.
        # Simplification: The input 'files' will be the set of lower-case paths?
        # No, let's keep the original paths in the list passed to this function.
        pass

    return linked_files

def analyze_links(root_dir):
    # 1. Find all MD files (store as dict: lower_path -> original_path)
    all_files_map = {}
    for root, dirs, files in os.walk(root_dir):
        if '.git' in dirs: dirs.remove('.git')
        if 'node_modules' in dirs: dirs.remove('node_modules')
        for file in files:
            if file.endswith('.md'):
                fpath = os.path.join(root, file)
                norm = normalize_path(fpath)
                all_files_map[norm] = fpath
    
    print(f"DEBUG: Found {len(all_files_map)} markdown files.")

    # 2. Scan content for links
    linked_paths = set()
    link_pattern = re.compile(r'\[([^\]]+)\]\(([^)]+)\)')

    for norm_path, real_path in all_files_map.items():
        try:
            with open(real_path, 'r', encoding='utf-8') as f:
                content = f.read()
        except Exception as e:
            print(f"Error reading {real_path}: {e}")
            continue

        matches = link_pattern.findall(content)
        for text, url in matches:
            if url.startswith(('http://', 'https://', 'mailto:', '#')):
                continue
            
            clean_url = url.split('#')[0].strip()
            if not clean_url:
                continue

            # Resolve relative link
            # real_path is 'C:\\...\\file.md'
            # url is '../other.md'
            try:
                # Use pathlib for resolution
                curr_dir = Path(real_path).parent
                target = (curr_dir / clean_url).resolve()
                target_norm = str(target).lower()
                
                if target_norm in all_files_map:
                    linked_paths.add(target_norm)
                else:
                    # Optional: Print broken links (we already did this, but good for debug)
                    # print(f"DEBUG: Link to {target_norm} not found in file list")
                    pass
            except Exception as e:
                pass
                
    return set(all_files_map.keys()), linked_paths, all_files_map

def main():
    root_dir = '.'
    all_files, linked_files, file_map = analyze_links(root_dir)
    
    orphans = all_files - linked_files
    
    # Filter known entry points
    entry_points = [
        'README.md', 
        'COURSE_MAP.md', 
        'SYLLABUS.md',
        'COURSE.md',
        'PROJECTS.md',
        'FINAL_CHALLENGE.md',
        'PREFLIGHT.md',
        'SQUID_GAMES.md',
        'LEARNING_LOG.md',
        'VERSION'
    ]
    
    for ep in entry_points:
        ep_norm = normalize_path(ep)
        if ep_norm in orphans:
            orphans.remove(ep_norm)

    if orphans:
        print(f"\nFound {len(orphans)} orphaned files (not linked from anywhere):")
        for orphan in sorted(orphans):
            # Print relative path from CWD
            real_path = file_map[orphan]
            try:
                rel = os.path.relpath(real_path, os.getcwd())
                print(f"- {rel}")
            except:
                print(f"- {real_path}")
    else:
        print("\nNo orphaned files found! All content is reachable.")

if __name__ == "__main__":
    main()