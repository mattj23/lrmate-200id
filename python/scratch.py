from copy import deepcopy


def main():
    d0 = {
        "item0": 10,
        "item1": [1, 2, 3],
    }

    d1 = deepcopy(d0)

    print(d0)
    print(d1)
    print("---")
    d1["item1"].append(4)
    print(d0)
    print(d1)


if __name__ == '__main__':

    main()