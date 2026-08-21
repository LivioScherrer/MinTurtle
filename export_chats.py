import json
import os

directory = r"C:\Users\livio\Livio Scherrer\Codes VS\Edu_V1.3\chat_exports"
output_file = r"C:\Users\livio\Livio Scherrer\Codes VS\Edu_V1.3\Alle_Chatverlaeufe_Komplett.txt"


def render_text(part):
    return part.get('text', '').replace('\n', ' ')


def render_tool(part):
    tool = part.get('tool', '')
    state = part.get('state', {}) or {}
    inp = state.get('input', {}) or {}
    out = state.get('output', '') or ''
    status = state.get('status', '')

    lines = []
    header = f"    [TOOL: {tool} | {status}]"
    lines.append(header)

    if tool in ('edit', 'edit_file', 'local-files_edit_file'):
        file_path = inp.get('filePath', '')
        old_str = inp.get('oldString', '')
        new_str = inp.get('newString', '')
        lines.append(f"      Datei: {file_path}")
        if old_str:
            lines.append("      VORHER (-):")
            for l in old_str.split('\n'):
                lines.append(f"        - {l}")
        if new_str:
            lines.append("      NACHHER (+):")
            for l in new_str.split('\n'):
                lines.append(f"        + {l}")
    elif tool == 'write' or 'write_file' in tool or 'local-files_write_file' in tool:
        file_path = inp.get('filePath', '')
        content = inp.get('content', '')
        lines.append(f"      Datei: {file_path}")
        lines.append(f"      Inhalt ({len(content.splitlines())} Zeilen):")
        for l in content.split('\n'):
            lines.append(f"        {l}")
    elif tool == 'bash' or tool == 'powershell':
        cmd = inp.get('command', '')
        workdir = inp.get('workdir', '')
        lines.append(f"      Befehl: {cmd}")
        if workdir:
            lines.append(f"      Verzeichnis: {workdir}")
        if out:
            lines.append("      Ausgabe:")
            for l in str(out).split('\n'):
                lines.append(f"        {l}")
    elif tool in ('glob', 'grep', 'read', 'local-files_read_text_file', 'local-files_read_file',
                  'local-files_list_directory', 'local-files_directory_tree', 'local-files_search_files'):
        if isinstance(inp, dict):
            for k, v in inp.items():
                if v is not None and v != '':
                    lines.append(f"      {k}: {v}")
        if out:
            lines.append("      Ausgabe:")
            for l in str(out).split('\n'):
                lines.append(f"        {l}")
    else:
        if isinstance(inp, dict):
            for k, v in inp.items():
                if isinstance(v, (str, int, float)) and v != '':
                    lines.append(f"      {k}: {str(v)[:500]}")
        if out and tool != 'to-do':
            out_s = str(out)
            lines.append("      Ausgabe:")
            for l in out_s.split('\n'):
                lines.append(f"        {l}")

    return '\n'.join(lines)


def render_patch(part):
    files = part.get('files', [])
    hash_val = part.get('hash', '')
    lines = [f"    [PATCH: {hash_val}]"]
    for f in files:
        lines.append(f"      - {f}")
    return '\n'.join(lines)


def format_part(role, part):
    ptype = part.get('type', '')
    if ptype == 'text':
        return f"[{role.upper()}]: {render_text(part)}\n"
    if ptype == 'tool':
        return f"{render_tool(part)}\n"
    if ptype == 'patch':
        return f"{render_patch(part)}\n"
    if ptype in ('reasoning',):
        return None
    if ptype in ('compaction', 'step-start', 'step-finish'):
        return None
    return None


def main():
    files = sorted([f for f in os.listdir(directory) if f.endswith('.json')])

    total_messages = 0
    total_tools = 0

    with open(output_file, 'w', encoding='utf-8') as f_out:
        f_out.write("# Gesamter Chatverlauf: Edu_V1.3\n")
        f_out.write(f"# Exportiert aus {len(files)} Sessions\n")
        f_out.write("# Enthält: alle Prompts (USER), alle Antworten (ASSISTANT) und alle Code-Änderungen (TOOL/PATCH)\n\n")

        for filename in files:
            filepath = os.path.join(directory, filename)
            f_out.write(f"\n{'='*80}\n--- Session: {filename} ---\n{'='*80}\n\n")
            try:
                with open(filepath, 'r', encoding='utf-16') as f_in:
                    data = json.load(f_in)
                messages = data.get('messages', [])
                total_messages += len(messages)
                for msg in messages:
                    role = msg.get('info', {}).get('role', 'unknown')
                    if role == 'user':
                        total_messages += 0
                    parts = msg.get('parts', [])
                    for part in parts:
                        rendered = format_part(role, part)
                        if rendered is None:
                            continue
                        if part.get('type') == 'tool':
                            total_tools += 1
                        f_out.write(rendered + '\n')
            except Exception as e:
                f_out.write(f"Error reading {filename}: {e}\n\n")

        f_out.write(f"\n{'='*80}\n")
        f_out.write(f"# ENDE DES EXPORTS\n")
        f_out.write(f"# {len(files)} Sessions, {total_messages} Nachrichten, {total_tools} Tool-Aufrufe\n")

    print(f"Fertig: {len(files)} Sessions -> {output_file}")
    print(f"Tool-Aufrufe gesamt: {total_tools}")


if __name__ == '__main__':
    main()