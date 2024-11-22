import edge_orchestrator.testbed_utils as tu

if __name__=='__main__':
    file = "/home/wifi/mn-wifi-cnet-vimemu-install/paper_testbed_code/map_talence/osm.passenger.rou.xml"

#    tu.setIdInRouXml(file=file, param="vehicle")

    #tu.setIdInXml(file,"trip","","10")
    tu.setIdInXmlWithStep(file,"vehicle",start=104, step=1)
    ids_after_modif = tu.getIdFromXml(file, "vehicle")

    print(ids_after_modif)