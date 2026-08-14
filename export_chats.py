import json
import os

directory = r"C:\Users\livio\Livio Scherrer\Codes VS\Edu_V1.3\chat_exports"
output_file = r"C:\Users\livio\Livio Scherrer\Codes VS\Edu_V1.3\Alle_Chatverlaeufe_Komplett.txt"

# List and sort files to try and group them nicely
files = sorted([f for f in os.listdir(directory) if f.endswith('.json')])

with open(output_file, 'w', encoding='utf-8') as f_out:
    for filename in files:
        filepath = os.path.join(directory, filename)
        f_out.write(f"\n--- Session: {filename} ---\n\n")
        try:
            with open(filepath, 'r', encoding='utf-16') as f_in:
                data = json.load(f_in)
                for msg in data.get('messages', []):
                    role = msg.get('info', {}).get('role', 'unknown')
                    parts = msg.get('parts', [])
                    for part in parts:
                        if part.get('type') == 'text':
                            text = part.get('text', '').replace('\n', ' ')
                            f_out.write(f"[{role.upper()}]: {text}\n\n")
        except Exception as e:
            f_out.write(f"Error reading {filename}: {e}\n\n")
