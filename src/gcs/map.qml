import QtQuick 2.5
import QtLocation 5.6
import MapItem 1.0
import QtPositioning 5.2

MapItem {
    id: mapItem

    signal waypointSelected(int index)
    signal mapDragEvent(int index, real clat, real clng, int type)
    signal updateHomeLocation(real clat, real clng)

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

        // list of navigation waypoints
        MapItemView {
            model: markerModel
            delegate: MapQuickItem {
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
                                    mapItem.mapDragEvent(wpIdx, drag.target.coordinate.latitude, drag.target.coordinate.longitude, 1)
                                }
                            }
                        }
                    }
                }
            }
        }

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
}
