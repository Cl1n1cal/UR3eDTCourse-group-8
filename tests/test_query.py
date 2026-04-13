from services.db_query_service import DBQueryService
from utils.configuration import load_config
import json

query_service = DBQueryService()
config = load_config("startup/startup.conf")
query_service.setup(influxdb_config=config["influxdb"])

training_data = query_service.get_all_values(
    start="-5m",
    window="500ms"
)

print(f"Retrieved {len(training_data)} records")
with open("training_data.json", "w") as file:
        json.dump(training_data, file, indent=4)
