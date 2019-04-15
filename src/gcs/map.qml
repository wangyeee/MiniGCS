import QtQuick 2.5
import QtLocation 5.9
import MapItem 1.0
import QtPositioning 5.6

MapItem {
    id: mapItem

    signal waypointSelected(int index)
    signal mapDragEvent(int index, real clat, real clng, int type)
    signal updateHomeLocation(real clat, real clng)
    signal mapCenterChangedEvent(real clat, real clng)
    signal mapZoomLevelChangedEvent(int zoom)

    Map {
        id: navMap
        gesture.enabled: true
        gesture.acceptedGestures: MapGestureArea.PinchGesture | MapGestureArea.PanGesture
        copyrightsVisible: false
        width: parent.width
        height: parent.height
        objectName: "navMap"
        zoomLevel: (maximumZoomLevel > minimumZoomLevel + 1) ? minimumZoomLevel + 1 : 1
        center {
            latitude: 0.0
            longitude: 0.0
        }
        plugin: Plugin {
            name: "osm"
        }

        MouseArea {
            id: mapArea
            anchors.fill: parent
            cursorShape: Qt.CrossCursor
            onDoubleClicked: {
                var c =  navMap.toCoordinate(Qt.point(mouse.x, mouse.y))
                markerModel.createWaypoint(c.latitude, c.longitude)  // create WP
                wpLine.addCoordinate(c)
            }
        }

        onCenterChanged: {
            mapItem.mapCenterChangedEvent(navMap.center.latitude, navMap.center.longitude)
        }

        onZoomLevelChanged: {
            mapItem.mapZoomLevelChangedEvent(navMap.zoomLevel)
        }

        // list of navigation waypoints
        MapItemView {
            model: markerModel
            delegate: MapItemGroup {
                MapQuickItem {
                    id: marker
                    anchorPoint.x: navDot.width / 2
                    anchorPoint.y: navDot.height / 2
                    coordinate: position
                    property int radius: 8
                    sourceItem: Rectangle {
                        id: navDot
                        width: marker.radius * 2
                        height: marker.radius * 2
                        color: dotColor
                        radius: marker.radius
                        antialiasing: true
                        opacity: 1
                        Text {
                            id: wpIdxTxt
                            text: (rowNumber + 1).toString()
                            font.family: "Arial"
                            anchors.centerIn: parent
                        }

                        MouseArea {
                            id: wpArea
                            anchors.fill: parent
                            onClicked: {
                                if (mouse.button == Qt.LeftButton) {
                                    // map the relative position of MapItemView to Map
                                    var wpIdx = parseInt(wpIdxTxt.text) - 1
                                    mapItem.waypointSelected(wpIdx)
                                }
                            }
                            drag {
                                target: marker
                                axis: Drag.XandYAxis
                                onActiveChanged: {
                                    var wpIdx = parseInt(wpIdxTxt.text) - 1
                                    if (drag.active) {
                                        mapItem.mapDragEvent(wpIdx, drag.target.coordinate.latitude, drag.target.coordinate.longitude, 0)
                                    } else {
                                        mapItem.mapDragEvent(wpIdx, drag.target.coordinate.latitude, drag.target.coordinate.longitude, 2)
                                    }
                                }
                            }
                        } // end MouseArea
                    } // end Rectangle
                } // end MapQuickItem
                MapCircle {
                    id: loiterRadiusCircle
                    visible: true
                    center: marker.coordinate
                    radius: loiterRadius
                    color: '#00000000'
                    border.width: 3
                    border.color: 'red'
                }
            } // end MapItemGroup
        } // end MapItemView

        MapItemView {
            model: adsbModel
            delegate: MapQuickItem {
                id: aircrafts
                anchorPoint.x: aircraftImage.width / 2
                anchorPoint.y: aircraftImage.height / 2
                coordinate: position
                sourceItem: Image {
                    id: aircraftImage
                    source: "res/aircraft.png"
                    transform: Rotation {
                        origin.x: aircraftImage.width / 2
                        origin.y: aircraftImage.height / 2
                        angle: heading
                    }
                }
            } // end MapQuickItem
        } // end MapItemView

        MapPolyline {
            id: wpLine
            objectName: "wpLine"
            line.width: 3
            line.color: "#90FF0000"
            antialiasing: true
            path: [
                {latitude: 0, longitude: 0}
            ]
        }

        // The home icon on map
        MapQuickItem {
            property bool firstRun: true

            id: homeMarker
            anchorPoint.x: homeImage.width / 4
            anchorPoint.y: homeImage.height
            // coordinate : navMap.center
            sourceItem: Image {
                id: homeImage
                source: "res/home.png"
                MouseArea {
                    id: homeArea
                    anchors.fill: parent
                    drag {
                        target: homeMarker
                        onActiveChanged: {
                            if (!drag.active) {
                                homeMarker.coordinate = drag.target.coordinate
                                wpLine.replaceCoordinate(0, homeMarker.coordinate)
                                mapItem.updateHomeLocation(drag.target.coordinate.latitude, drag.target.coordinate.longitude)
                            }
                        }
                    }
                }
            }
        }

        // Drone symbol
        MapQuickItem {
            id: droneMarker
            anchorPoint.x: droneDot.width / 2
            anchorPoint.y: droneDot.height / 2
            coordinate : navMap.center
            visible: false
            sourceItem: Rectangle {
                id: droneDot
                width: 12
                height: 12
                color: "red"
                radius: 6
                antialiasing: true
                opacity: 1
            }
        }
        MapCircle {
            id: droneRadius
            visible: false
            center: navMap.center
            radius: 100.0
            color: '#800000FF'
            border.width: 1
        }
    }

    onUpdateZoomLevel: {
        if (zoom >= navMap.minimumZoomLevel || zoom <= navMap.maximumZoomLevel) {
            navMap.zoomLevel = zoom
        }
    }

    onUpdateDroneLocation: {
        droneMarker.coordinate.latitude = lat
        droneMarker.coordinate.longitude = lng
        droneRadius.center.latitude = lat
        droneRadius.center.longitude = lng
        droneRadius.radius = hacc
        droneMarker.visible = true
        droneRadius.visible = (hacc > 5.0)
    }

    onUpdateCoordinate: {
        navMap.center.latitude = lat
        navMap.center.longitude = lng
        if (homeMarker.firstRun) {
            homeMarker.coordinate.latitude = lat
            homeMarker.coordinate.longitude = lng
            wpLine.replaceCoordinate(0, homeMarker.coordinate)
            homeMarker.firstRun = false
        }
    }

    onUpdateHomeCoordinate: {
        homeMarker.coordinate.latitude = lat
        homeMarker.coordinate.longitude = lng
        wpLine.replaceCoordinate(0, homeMarker.coordinate)
    }

    onWaypointRemoved: {
        wpLine.removeCoordinate(wpNumber + 1)  // 0 for home
    }

    onWaypointChanged: {
        wpLine.replaceCoordinate(wpNumber + 1, QtPositioning.coordinate(latitude, longitude))
    }

    onWaypointChangedInt: {
        var c = navMap.toCoordinate(navMap.mapFromGlobal(x, y))
        wpLine.replaceCoordinate(wpNumber + 1, c)
        mapItem.mapDragEvent(wpNumber, c.latitude, c.longitude, 1)
    }

    onWaypointCreated: {
        wpLine.addCoordinate(QtPositioning.coordinate(lat, lng))
    }

    onAllPolylineRemoved: {
        for (var i = wpLine.path.length - 1; i > 0; i--) {
            wpLine.removeCoordinate(i)
        }
    }
}
