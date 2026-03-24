import random
from enum import StrEnum, auto
from collections import Counter

class Weather(StrEnum):
    SUNNY = auto()
    CLOUDY = auto()
    RAINY = auto()

transitions = {
    Weather.SUNNY: {Weather.SUNNY: 0.8, Weather.CLOUDY: 0.2, Weather.RAINY: 0.0},
    Weather.CLOUDY: {Weather.SUNNY: 0.4, Weather.CLOUDY: 0.4, Weather.RAINY: 0.2},
    Weather.RAINY: {Weather.SUNNY: 0.2, Weather.CLOUDY: 0.6, Weather.RAINY: 0.2},
}

def weather_sim(init_weather, days):
    current_weather = init_weather
    sequence = [current_weather]

    for _ in range(days - 1):
        next_weather = random.choices(
            population=list(transitions[current_weather].keys()),
            weights=list(transitions[current_weather].values()),
            k=1
        )[0]
        sequence.append(next_weather)
        current_weather = next_weather

    return sequence

# 2-b
# result = weather_sim(Weather.SUNNY, 10)
# print("Generated weather sequence:")
# for i, weather in enumerate(result, start=1):
#     print(f"Day {i}: {weather.value}")

# 2-c
total_days = 1_000_000
simulated_data = weather_sim(Weather.SUNNY, total_days)

# 計算各狀態出現次數與比例
counts = Counter(simulated_data)
stationary_dist = {state: count / total_days for state, count in counts.items()}

print(f"Stationary Distribution after {total_days} days:")
for state in Weather:
    print(f"{state.value.capitalize()}: {stationary_dist[state]:.4f}")
