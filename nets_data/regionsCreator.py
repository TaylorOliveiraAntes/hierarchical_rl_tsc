import json

if __name__ == '__main__':

  #regions = dict()
  # regions["R11"] = {"level": 1, "sub": ["A3", "B3", "A2", "B2"]}
  # regions["R12"] = {"level": 1, "sub": ["C3", "D3", "C2", "D2"]}
  # regions["R13"] = {"level": 1, "sub": ["A1", "B1", "A0", "B0"]}
  # regions["R14"] = {"level": 1, "sub": ["C1", "D1", "C0", "D0"]}

  # regions = [ {"R11": ["A3", "B3", "A2", "B2"],
  #             "R12": ["C3", "D3", "C2", "D2"],
  #             "R13": ["A1", "B1", "A0", "B0"],
  #             "R14": ["C1", "D1", "C0", "D0"]}
  #             ,
  #             {"R21": ["R11", "R12", "R13", "R14"]}
  #           ]

  regions = {
  "Order": [ ["R11", "R12", "R13", "R14"], ["R21"] ],
  "RegionsDict":{
      "R11": { "subordinates": ["A3", "B3", "A2", "B2"] },
      "R12": { "subordinates": ["C3", "D3", "C2", "D2"] },
      "R13": { "subordinates": ["A1", "B1", "A0", "B0"] },
      "R14": { "subordinates": ["C1", "D1", "C0", "D0"] },
      "R21": { "subordinates": ["R11", "R12", "R13", "R14"] }
    }
  }

  filename = "4x4_regions.json"
  with open(filename, 'w') as file:
    json.dump(regions, file, indent = 2)