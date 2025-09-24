# configs/CARLA/test_suites/town02_one_lap.py

TEST_ROUTES = [
    {
        "map": "Town02",
        "start": "160.0, -105.3, 0.42, 0.00, 0.00, 180.00",
        "end": "165.0, -105.3",  
        "distance": 400.0,       
        "commands": ["Straight", "Left", "Straight", "Left", "Straight", "Straight", "Right", "Straight"],  
    }
]
