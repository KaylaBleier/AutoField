# test_ui_input.py
import json
import os

FT_TO_M = 0.3048
M_TO_FT = 1 / FT_TO_M

def ask_user_id():
    while True:
        uid = input("User ID (2 digits, e.g. 07): ").strip()
        if uid.isdigit() and len(uid) == 2:
            return uid
        print("Please enter exactly 2 digits (00–99).")

def ask_number(prompt):
    while True:
        try:
            val = float(input(prompt).strip())
            if val <= 0:
                print("Must be > 0.")
                continue
            return val
        except ValueError:
            print("Enter a valid number.")

def ask_unit(prompt):
    while True:
        u = input(prompt).strip().lower()
        if u in ["m", "meter", "meters"]:
            return "m"
        if u in ["ft", "foot", "feet"]:
            return "ft"
        print("Unit must be m or ft.")

def to_meters(value, unit):
    return value if unit == "m" else value * FT_TO_M

def to_feet(value, unit):
    return value if unit == "ft" else value * M_TO_FT

def get_field_spec_cli():
    print("\n=== AutoField Terminal Input ===\n")

    user_id = ask_user_id()

    length_val = ask_number("Length value: ")
    length_unit = ask_unit("Length unit (m/ft): ")

    width_val = ask_number("Width value: ")
    width_unit = ask_unit("Width unit (m/ft): ")

    length_m = to_meters(length_val, length_unit)
    width_m  = to_meters(width_val, width_unit)

    length_ft = to_feet(length_val, length_unit)
    width_ft  = to_feet(width_val, width_unit)

    field_spec = {
        "user_id": user_id,
        "inputs": {
            "length": {"value": length_val, "unit": length_unit},
            "width":  {"value": width_val,  "unit": width_unit},
        },
        "converted": {
            "meters": {"length_m": length_m, "width_m": width_m},
            "feet":   {"length_ft": length_ft, "width_ft": width_ft},
        }
    }

    out_dir = "ui_output"
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"field_spec_{user_id}.json")

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(field_spec, f, indent=2)

    print("\nSaved:", out_path)
    print(json.dumps(field_spec, indent=2))
    return field_spec

def main():
    get_field_spec_cli()

if __name__ == "__main__":
    main()
