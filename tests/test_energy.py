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

    assert round(energy_model.energy(71.4142842854285, 1, False), 5) == 3535.98773
    assert round(energy_model.energy(71.4142842854285, 2, False), 5) == 1063.71084
    assert round(energy_model.energy(71.4142842854285, 10, False), 5) == 630.50208
    assert round(energy_model.energy(71.4142842854285, 10, True), 5) == 1319.14278

