import fanet.energy_model as energy_model

def test_energy() -> None:
    """Tests the energy model"""
    energy = energy_model.energy(100, 1000)
    assert energy != None
    energy =  energy_model.energy(0, 0)
    assert energy == 0
    energy = energy_model.energy(0, 1000, hover=False)
    assert energy == 0
    energy = energy_model.energy(100, 1000, hover=False)
    assert energy != None

