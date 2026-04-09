import json

def round_data(filename, output_filename):
    with open(filename, "r") as file:
        data = json.load(file)
    
    for item in data:
        for key in item:
            if key != "time_stamp" and isinstance(item[key], (int, float)):
                item[key] = round(item[key], 4)
    
    with open(output_filename, "w") as file:
        json.dump(data, file, indent=4)
    
    print(f"Processed {filename} → {output_filename}")

# Process both files
round_data("mockup_results.json", "mockup_results_rounded.json")
round_data("sim_results.json", "sim_results_rounded.json")