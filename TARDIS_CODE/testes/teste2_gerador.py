# gerador.py
import time

def main():
    contador = 0
    while True:
        print(f"Dado gerado: {contador}")
        contador += 1
        time.sleep(2)


if __name__ == "__main__":
    main()