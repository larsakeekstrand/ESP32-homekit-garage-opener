#!/usr/bin/env python3
"""Regenerate the embedded web pages in main/web_server.c from tools/web/*.html.

The device's web UI is served from two string literals (SETUP_HTML, PAGE_HTML) in
main/web_server.c. Editing escaped C strings by hand is error-prone, so the source
of truth lives as real HTML in tools/web/. Workflow:

    1. edit tools/web/page.html or tools/web/setup.html
    2. run  python3 tools/gen_web_page.py
    3. rebuild (idf.py build)
"""
import re
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def clit(name, s):
    s = s.replace("\\", "\\\\").replace('"', '\\"')
    body = "\n".join('"%s\\n"' % ln for ln in s.split("\n"))
    return "static const char %s[] =\n%s;\n" % (name, body)


def main():
    setup = open(os.path.join(ROOT, "tools/web/setup.html"), encoding="utf-8").read().rstrip("\n")
    page = open(os.path.join(ROOT, "tools/web/page.html"), encoding="utf-8").read().rstrip("\n")
    ws_path = os.path.join(ROOT, "main/web_server.c")
    ws = open(ws_path, encoding="utf-8").read()

    region = re.compile(r"(// ---- static pages[^\n]*\n\n)(.*?)(\n// ---- helpers)", re.S)
    m = region.search(ws)
    assert m, "static-pages region not found in web_server.c"
    mid = ('\n// First-run "create password" page (served while no password is set).\n'
           + clit("SETUP_HTML", setup)
           + '\n// Main status/config page. Values are filled by JS from /api/status.\n'
           + clit("PAGE_HTML", page))
    ws = ws[:m.start()] + m.group(1) + mid.rstrip("\n") + "\n" + m.group(3) + ws[m.end():]
    open(ws_path, "w", encoding="utf-8").write(ws)
    print("regenerated PAGE_HTML(%d B) SETUP_HTML(%d B) into main/web_server.c" % (len(page), len(setup)))


if __name__ == "__main__":
    main()
