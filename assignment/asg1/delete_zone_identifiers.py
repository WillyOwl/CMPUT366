#!/usr/bin/env python3
"""
Script to delete all Zone.Identifier files from the dao-map directory.
These files are created by Windows when downloading files from the internet
and are not needed on Linux systems.
"""

import os
import glob

def delete_zone_identifiers():
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dao_map_dir = os.path.join(script_dir, 'dao-map')
    
    # Check if dao-map directory exists
    if not os.path.exists(dao_map_dir):
        print(f"Directory {dao_map_dir} does not exist!")
        return
    
    # Find all Zone.Identifier files
    zone_files = glob.glob(os.path.join(dao_map_dir, '*:Zone.Identifier'))
    
    if not zone_files:
        print("No Zone.Identifier files found.")
        return
    
    print(f"Found {len(zone_files)} Zone.Identifier files:")
    
    # List all files that will be deleted
    for file_path in zone_files:
        print(f"  - {os.path.basename(file_path)}")
    
    # Ask for confirmation
    response = input(f"\nDelete all {len(zone_files)} Zone.Identifier files? (y/N): ")
    
    if response.lower() in ['y', 'yes']:
        deleted_count = 0
        for file_path in zone_files:
            try:
                os.remove(file_path)
                deleted_count += 1
                print(f"Deleted: {os.path.basename(file_path)}")
            except OSError as e:
                print(f"Error deleting {os.path.basename(file_path)}: {e}")
        
        print(f"\nSuccessfully deleted {deleted_count} out of {len(zone_files)} Zone.Identifier files.")
    else:
        print("Operation cancelled.")

if __name__ == "__main__":
    delete_zone_identifiers()