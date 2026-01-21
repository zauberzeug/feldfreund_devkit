import importlib
import inspect
import logging
import re
import sys
from pathlib import Path
from types import ModuleType

import mkdocs_gen_files

nav = mkdocs_gen_files.Nav()


def extract_events(filepath: str) -> dict[str, str]:
    with open(filepath, encoding='utf-8') as f:
        lines = f.read().splitlines()
    events_: dict[str, str] = {}
    for i, line in enumerate(lines):
        if re.search(r'= Event(\[.*?\])?\(\)$', line):
            event_name_ = line.strip().split()[0].removeprefix('self.').rstrip(':')
            event_doc_ = lines[i + 1].split('"""')[1]
            events_[event_name_] = event_doc_
    return events_


def format_type(hint) -> str:
    if hint is None:
        return ''
    if hint is type(None):
        return 'None'
    if hasattr(hint, '__name__'):
        return f'`{hint.__name__}`'
    type_str = str(hint).replace('typing.', '').replace('NoneType', 'None')
    return f'`{type_str}`'


def extract_properties(cls: type) -> dict[str, tuple[str, str]]:
    props: dict[str, tuple[str, str]] = {}
    for name, obj in inspect.getmembers(cls):
        if name.startswith('_'):
            continue
        if isinstance(obj, property) and obj.fget is not None:
            doc = obj.fget.__doc__ or ''
            doc = doc.strip().split('\n')[0] if doc else ''
            type_hint = obj.fget.__annotations__.get('return')
            props[name] = (format_type(type_hint), doc)
    return props


def extract_instance_attributes(filepath: str) -> dict[str, tuple[str, str]]:
    """Extract instance attributes with their inline docstrings from source."""
    with open(filepath, encoding='utf-8') as f:
        lines = f.read().splitlines()
    attrs: dict[str, tuple[str, str]] = {}
    for i, line in enumerate(lines):
        match = re.match(r'\s+self\.(\w+)(?::\s*(\S+))?\s*=', line)
        if match:
            attr_name = match.group(1)
            attr_type = match.group(2) or ''
            if attr_name.startswith('_'):
                continue
            if i + 1 < len(lines) and '"""' in lines[i + 1]:
                doc = lines[i + 1].split('"""')[1]
                type_str = f'`{attr_type}`' if attr_type else ''
                attrs[attr_name] = (type_str, doc)
    return attrs


for path in sorted(Path('feldfreund_devkit').rglob('__init__.py')):
    identifier = str(path.parent).replace('/', '.')
    if identifier == 'feldfreund_devkit':
        continue

    try:
        module = importlib.import_module(identifier)
    except Exception:
        logging.exception('Failed to import %s', identifier)
        sys.exit(1)

    doc_path = path.parent.with_suffix('.md')
    found_something = False
    for name in getattr(module, '__all__', dir(module)):
        if name.startswith('_'):
            continue
        cls = getattr(module, name)
        if isinstance(cls, ModuleType):
            continue
        if not inspect.isclass(cls):
            continue
        if not cls.__doc__:
            continue
        source_file = inspect.getfile(cls)
        events = extract_events(source_file)
        properties = extract_properties(cls)
        instance_attrs = extract_instance_attributes(source_file)
        if cls.__name__ != name:
            cls_module = cls.__module__
            doc_identifier = f'{cls_module}.{cls.__name__}'
        else:
            doc_identifier = f'{identifier}.{name}'
        properties = {k: v for k, v in properties.items() if v[1]}
        instance_attrs = {k: v for k, v in instance_attrs.items() if v[1] and k not in events}
        members = {**instance_attrs, **properties}
        filters = list(events.keys()) + list(members.keys())
        with mkdocs_gen_files.open(Path('reference', doc_path), 'a') as fd:
            print(f'::: {doc_identifier}', file=fd)
            if filters:
                print('    options:', file=fd)
                print('      filters:', file=fd)
                for filter_name in filters:
                    print(f'        - "!{filter_name}"', file=fd)
            if members:
                print('### Attributes & Properties', file=fd)
                print('Name | Type | Description', file=fd)
                print('- | - | -', file=fd)
                for member_name, (member_type, member_doc) in members.items():
                    print(f'`{member_name}` | {member_type} | {member_doc}', file=fd)
                print('', file=fd)
            if events:
                print('### Events', file=fd)
                print('Name | Description', file=fd)
                print('- | -', file=fd)
                for event_name, event_doc in events.items():
                    print(f'`{event_name}` | {event_doc}', file=fd)
                print('', file=fd)
        found_something = True

    if found_something:
        nav[path.parent.parts[1:]] = doc_path.as_posix()

with mkdocs_gen_files.open('reference/SUMMARY.md', 'w') as nav_file:
    nav_file.writelines(nav.build_literate_nav())
