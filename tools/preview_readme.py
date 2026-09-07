"""Собрать страницу проекта в HTML — посмотреть до того, как выкладывать.

Гитхаб покажет README только после публикации, а увидеть, что получилось,
хочется раньше. Здесь тот же файл собирается в страницу с оформлением,
похожим на гитхабовское, и открывается в браузере.

Картинки, которых ещё нет, показываются рамкой с именем файла — сразу
видно, чего не хватает и куда это класть.

    ./.venv/bin/python tools/preview_readme.py
"""

from __future__ import annotations

import argparse
import webbrowser
from pathlib import Path

КОРЕНЬ = Path(__file__).resolve().parents[1]

СТИЛЬ = """
:root { color-scheme: light dark; }
body { margin: 0 auto; max-width: 900px; padding: 32px 24px 96px;
       font: 16px/1.6 -apple-system, BlinkMacSystemFont, "Segoe UI",
             Helvetica, Arial, sans-serif;
       color: #1f2328; background: #fff; }
h1 { font-size: 2em; padding-bottom: .3em; border-bottom: 1px solid #d0d7de; }
h2 { font-size: 1.5em; margin-top: 2em; padding-bottom: .3em;
     border-bottom: 1px solid #d0d7de; }
img { max-width: 100%; }
table { border-collapse: collapse; width: 100%; margin: 1em 0; display: block;
        overflow-x: auto; }
th, td { border: 1px solid #d0d7de; padding: 8px 13px; text-align: left; }
tr:nth-child(2n) { background: #f6f8fa; }
code { background: #f0f1f2; padding: .2em .4em; border-radius: 6px;
       font-size: 85%; }
pre { background: #f6f8fa; padding: 16px; border-radius: 6px; overflow: auto; }
pre code { background: none; padding: 0; }
a { color: #0969da; text-decoration: none; }
a:hover { text-decoration: underline; }
hr { border: 0; border-top: 1px solid #d0d7de; margin: 24px 0; }
.нет-файла { display: block; padding: 40px 20px; margin: 12px 0;
             border: 2px dashed #d0d7de; border-radius: 8px; text-align: center;
             color: #59636e; background: #f6f8fa; font-size: 14px; }
.нет-файла b { display: block; font-size: 16px; color: #1f2328;
               margin-bottom: 6px; }
@media (prefers-color-scheme: dark) {
  body { color: #e6edf3; background: #0d1117; }
  h1, h2 { border-color: #30363d; }
  th, td { border-color: #30363d; }
  tr:nth-child(2n) { background: #161b22; }
  code, pre { background: #161b22; }
  a { color: #4493f8; }
  hr { border-color: #30363d; }
  .нет-файла { border-color: #30363d; background: #161b22; color: #9198a1; }
  .нет-файла b { color: #e6edf3; }
}
"""


def подменить_картинки(html: str) -> tuple[str, list[str]]:
    """Отсутствующие картинки заменить рамкой с именем файла.

    Иначе браузер покажет сломанный значок, и по нему не понять, то ли
    файла нет, то ли ссылка неверная.
    """
    import re

    пропало: list[str] = []

    def замена(совпадение: re.Match) -> str:
        тег = совпадение.group(0)
        путь = совпадение.group(1)
        if путь.startswith(("http://", "https://")) or (КОРЕНЬ / путь).exists():
            return тег
        пропало.append(путь)
        подпись = re.search(r'alt="([^"]*)"', тег)
        текст = подпись.group(1) if подпись else ""
        return (f'<span class="нет-файла"><b>нет файла: {путь}</b>'
                f'{текст}</span>')

    return re.sub(r'<img[^>]*src="([^"]+)"[^>]*>', замена, html), пропало


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--файл", type=Path, default=КОРЕНЬ / "README.md")
    p.add_argument("--не-открывать", action="store_true")
    args = p.parse_args()

    import markdown

    текст = args.файл.read_text(encoding="utf-8")
    тело = markdown.markdown(текст, extensions=["tables", "fenced_code"])
    тело, пропало = подменить_картинки(тело)

    страница = (f"<!doctype html><html lang=ru><head><meta charset=utf-8>"
                f"<title>{args.файл.name}</title><style>{СТИЛЬ}</style>"
                f"</head><body>{тело}</body></html>")

    куда = КОРЕНЬ / "docs" / "media" / "просмотр.html"
    куда.write_text(страница, encoding="utf-8")

    print(f"страница собрана: {куда}")
    if пропало:
        print(f"не хватает картинок ({len(пропало)}):")
        for путь in пропало:
            print(f"  {путь}")
        print("положите их — и страница соберётся целиком")
    else:
        print("все картинки на месте")

    if not args.не_открывать:
        webbrowser.open(куда.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
