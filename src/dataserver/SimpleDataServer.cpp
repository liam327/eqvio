/*
    This file is part of EqVIO.

    EqVIO is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EqVIO is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EqVIO.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "eqvio/dataserver/SimpleDataServer.h"

MeasurementType SimpleDataServer::nextMeasurementType() const {

    //check to see if we have all types of data. In this case compare times 
    if (nextImageData && nextIMUData && nextAttitudeData) {
        if ((nextImageData.stamp <= IMUQueue.front().stamp) && (nextImageData.stamp <= nextAttitudeData.stamp)) {
            return MeasurementType::Image;
        } 
        else if ((nextIMUData.stamp <= nextImageData.stamp) && (nextIMUData.stamp <= nextAttitudeData.stamp)){
            return MeasurementType::IMU;
        }
        else {
            return MeasurementType::Attitude;
        }
    } 

    //check to see if we have two type of data 
    if (nextImageData && nextIMUData) {
        return (nextImageData.stamp <= nextIMUData.stamp) ? MeasurementType::Image : MeasurementType::IMU;
    }
    else if (nextImageData && nextAttitudeData) {
        return (nextImageData.stamp <= nextAttitudeData.stamp) ? MeasurementType::Image : MeasurementType::Attitude;
    }
    else if (nextIMUData && nextAttitudeData) {
        return (nextIMUData.stamp <= nextAttitudeData.stamp) ? MeasurementType::IMU : MeasurementType::Attitude;
    }
    
    else if (nextImageData) {
        return MeasurementType::Image;
    } 
    else if (nextIMUData) {
        return MeasurementType::IMU;
    } 
    else if (nextAttitudeData) {
        return MeasurementType::Attitude;
    }
    return MeasurementType::None;
}

StampedImage SimpleDataServer::getImage() {
    StampedImage retImageData = *nextImageData;
    nextImageData = datasetReaderPtr->nextImage();
    return retImageData;
}

IMUVelocity SimpleDataServer::getIMU() {
    IMUVelocity retIMUData = *nextIMUData;
    nextIMUData = datasetReaderPtr->nextIMU();
    return retIMUData;
}

StampedAttiude SimpleDataServer::getAttitude() {
    StampedAttiude retAttitudeData = *nextAttitudeData;
    nextAttitudeData = datasetReaderPtr->nextAttitude();
    return retAttitudeData;
}

SimpleDataServer::SimpleDataServer(std::unique_ptr<DatasetReaderBase>&& datasetReader)
    : DataServerBase(std::move(datasetReader)) {
    nextImageData = datasetReaderPtr->nextImage();
    nextIMUData = datasetReaderPtr->nextIMU();
    nextAttitudeData = datasetReaderPtr->nextAttitude();
}

double SimpleDataServer::nextTime() const {
    MeasurementType nextMT = nextMeasurementType();
    if (nextMT == MeasurementType::Image) {
        return nextImageData.stamp;
    } 
    else if (nextMT == MeasurementType::IMU) {
        return nextIMUData.stamp;
    }
    else if (nextMT == MeasurementType::Attitude) {
        return nextAttitudeData.stamp;
    }
    return std::nan("");
}