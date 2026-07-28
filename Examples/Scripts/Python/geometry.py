#!/usr/bin/env python3

import os
import json
from pathlib import Path

import acts
import acts.examples
from acts.json import MaterialMapJsonConverter
from acts.examples.odd import getOpenDataDetector
from acts.examples import (
    WhiteBoard,
    AlgorithmContext,
    ProcessCode,
    CsvTrackingGeometryWriter,
    ObjTrackingGeometryWriter,
)

from acts.examples.json import (
    JsonSurfacesWriter,
    JsonMaterialWriter,
    JsonFormat,
)


def runGeometry(
    trackingGeometry,
    decorators,
    outputDir: Path,
    events=1,
    outputObj=True,
    outputCsv=True,
    outputJson=True,
):
    for ievt in range(events):
        eventStore = WhiteBoard(name=f"EventStore#{ievt}", level=acts.logging.INFO)
        ialg = 0
        ithread = 0

        context = AlgorithmContext(ialg, ievt, eventStore, ithread)

        for cdr in decorators:
            r = cdr.decorate(context)
            if r != ProcessCode.SUCCESS:
                raise RuntimeError("Failed to decorate event context")

        if outputCsv:
            # if not os.path.isdir(outputDir / "csv"):
            #    os.makedirs(outputDir / "csv")
            writer = CsvTrackingGeometryWriter(
                level=acts.logging.INFO,
                trackingGeometry=trackingGeometry,
                outputDir=str(outputDir / "csv"),
                writePerEvent=True,
            )
            writer.write(context)

        if outputObj:
            writer = ObjTrackingGeometryWriter(
                level=acts.logging.INFO, outputDir=outputDir / "obj"
            )
            writer.write(context, trackingGeometry)

        if outputJson:
            print("json surface writer")
            # if not os.path.isdir(outputDir / "json"):
            #    os.makedirs(outputDir / "json")
            writer = JsonSurfacesWriter(
                level=acts.logging.INFO,
                trackingGeometry=trackingGeometry,
                outputDir=str(outputDir / "json"),
                writePerEvent=True,
                writeSensitive=True,
            )
            writer.write(context)
            print("jmconvertercfg")
            jmConverterCfg = MaterialMapJsonConverter.Config(
                processSensitives=True,
                processApproaches=True,
                processRepresenting=True,
                processBoundaries=True,
                processVolumes=True,
                processNonMaterial=True,
                context=context.geoContext,
            )
            print("jsonmaterialwriter")
            jmw = JsonMaterialWriter(
                level=acts.logging.VERBOSE,
                converterCfg=jmConverterCfg,
                fileName=str(outputDir / "geometry-map"),
                writeFormat=JsonFormat.Json,
            )

            jmw.write(trackingGeometry)
            print("Finished")

if "__main__" == __name__:
    jsonFile="/cvmfs/sphenix.sdcc.bnl.gov/calibrations/sphnxpro/cdb/ACTSGEOMETRYCONFIG/79/0e/790e3c6de619a65cce779262022539b1_tgeo-sphenix-mms-actsv45.0.0.json"
    tgeo_fileName = "/sphenix/user/jdosbo/git/sphenix/reco_geometry_macros/detectors/sPHENIX/sPHENIXActsGeom.root"
    customLogLevel = acts.examples.defaultLogging(logLevel=acts.logging.INFO)
    from acts.examples.tgeo import TGeoDetector
    # detector = acts.examples.GenericDetector()
    config = TGeoDetector.Config()

    config.fileName = tgeo_fileName
    config.surfaceLogLevel = acts.logging.INFO
    config.layerLogLevel = acts.logging.INFO
    config.volumeLogLevel = acts.logging.INFO
    config.readJson(str(jsonFile))
    detector = TGeoDetector(config)
    trackingGeometry = detector.trackingGeometry()
    decorators = detector.contextDecorators()

    runGeometry(trackingGeometry, decorators, outputDir=Path.cwd())

    # Uncomment if you want to create the geometry id mapping for DD4hep
    # dd4hepIdGeoIdMap = acts.examples.dd4hep.createDD4hepIdGeoIdMap(trackingGeometry)
    # dd4hepIdGeoIdValueMap = {}
    # for key, value in dd4hepIdGeoIdMap.items():
    #     dd4hepIdGeoIdValueMap[key] = value.value

    # with open('odd-dd4hep-geoid-mapping.json', 'w') as outfile:
    #    json.dump(dd4hepIdGeoIdValueMap, outfile)
