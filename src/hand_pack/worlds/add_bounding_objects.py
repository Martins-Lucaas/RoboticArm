#!/usr/bin/env python3
import re
import sys

def main(input_proto, output_proto):
    text = open(input_proto, encoding='utf-8').read()

    # Bloco de boundingObject a ser inserido
    bb = """
      boundingObject Shape {
        appearance USE defaultAppearance
        geometry Box {
          size 0.05 0.05 0.05
        }
      }"""

    # Regex: encontra cada bloco 'physics Physics { ... }' e o substitui por si mesmo + boundingObject
    pattern = re.compile(r'(physics\s+Physics\s*\{[^}]*\})', flags=re.DOTALL)
    new_text, count = pattern.subn(lambda m: m.group(1) + bb, text)

    print(f"✅ Inseridos {count} boundingObjects de 0.05×0.05×0.05")
    with open(output_proto, 'w', encoding='utf-8') as f:
        f.write(new_text)
    print(f"Arquivo gerado: {output_proto}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Uso: python3 add_bounding_objects.py entrada.proto saída.proto")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
