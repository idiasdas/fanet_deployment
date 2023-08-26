import os, sys
this_dirctory = os.path.dirname(__file__)
sys.path.append( this_dirctory + '/../src/')
import energy_model

def test_energy():
    energy = energy_model.energy(100, 1000)
    assert energy != None
    energy =  energy_model.energy(0, 0)
    assert energy == 0
    energy = energy_model.energy(0, 1000, hover=False)
    assert energy == 0
    energy = energy_model.energy(100, 1000, hover=False)
    assert energy != None

