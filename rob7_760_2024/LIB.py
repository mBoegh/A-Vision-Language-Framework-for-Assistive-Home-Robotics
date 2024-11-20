import json
import logging
import os



class JSON_Handler:
    """
    Class for handling loading keys for each respective ROS2 node, subkeys for each setting and the value of each setting. 
    """
    
    def __init__(self, json_file_path):
        
        self.json_file_path = json_file_path
        self.json_obj = None

        try:
            self.json_obj = json.load(open(self.json_file_path, 'r'))
        except json.JSONDecodeError as error:
            print(f"DEBUG @ script 'EXOLIB.py' class 'JSON_Handler' function '__init__'; SYSTEM MESSAGE: Failed loading .json file with error: {error}")
            return None  # Return None if the input is not valid JSON


    def get_keys(self):
        """
        Function for getting all top level keys in a json file.
        """

        if isinstance(self.json_obj, dict):
            return list(self.json_obj.keys())
        else:                
            return None  # Return None if the input is not a JSON object
    

    def get_sublevel_keys(self, key):
        """
        Function for getting all sublevel keys of a specified key in a json file.
        """
            
        if isinstance(self.json_obj, dict):
            if key in self.json_obj and isinstance(self.json_obj[key], dict):
                return list(self.json_obj[key].keys())
            else:
                return None  # Sublevel key not found or is not a dictionary
        else:
            return None  # Input is not a JSON object
    

    def get_subkey_value(self, top_level_key, subkey):
        """
        Function for getting the value of a specified subkey of a key in a JSON file.
        """
        if isinstance(self.json_obj, dict):
            if top_level_key in self.json_obj and isinstance(self.json_obj[top_level_key], dict):
                if subkey in self.json_obj[top_level_key]:
                    return self.json_obj[top_level_key][subkey]
                else:
                    return None  # Subkey not found
            else:
                return None  # Top-level key not found or is not a dictionary
        else:
            return None  # Input is not a JSON object


    def set_subkey_value(self, top_level_key, subkey, value):
        """
        Function for setting the value of a specified subkey of a key in a JSON file.
        """
        if isinstance(self.json_obj, dict):
            if top_level_key in self.json_obj and isinstance(self.json_obj[top_level_key], dict):
                self.json_obj[top_level_key][subkey] = value
                # Save changes back to the JSON file
                try:
                    with open(self.json_file_path, 'w') as file:
                        json.dump(self.json_obj, file, indent=4)
                    return True  # Successfully set the value
                except Exception as e:
                    print(f"Error saving changes to JSON file: {e}")
                    return False  # Failed to save changes
            else:
                return False  # Top-level key not found or is not a dictionary
        else:
            return False  # Input is not a JSON object

