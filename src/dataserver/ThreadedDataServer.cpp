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

#include "eqvio/dataserver/ThreadedDataServer.h"

MeasurementType ThreadedDataServer::nextMeasurementType() const {
    std::unique_lock lck(ioMutex);
    queuesReady_cv.wait(
        lck, [this] { return (IMUDataFinished || !IMUQueue.empty()) && (imageDataFinished || !imageQueue.empty()) && (AttitudeDataFinished || !AttitudeQueue.empty()); });

    // Check if any of the data is finished
    if (imageQueue.empty() && IMUQueue.empty() && IMUQueue.empty()) {
        return MeasurementType::None;
    } 
    //check to see if two of the queues are empty 
    else if (imageQueue.empty() && !IMUQueue.empty() && AttitudeQueue.empty()) {
        return MeasurementType::IMU;
    } 
    else if (!imageQueue.empty() && IMUQueue.empty() && AttitudeQueue.empty()) {
        return MeasurementType::Image;
    }
    else if (imageQueue.empty() && IMUQueue.empty() && !AttitudeQueue.empty()) {
        return MeasurementType::Attitude;
    }

    //check to see if one of the queues is empty  
    if (imageQueue.empty() && !IMUQueue.empty() && !AttitudeQueue.empty()) {
        return (AttitudeQueue.front().stamp <= IMUQueue.front().stamp) ? MeasurementType::Attitude : MeasurementType::IMU;
    } 
    else if (!imageQueue.empty() && IMUQueue.empty() && !AttitudeQueue.empty()) {
        return (AttitudeQueue.front().stamp <= imageQueue.front().stamp) ? MeasurementType::Attitude : MeasurementType::Image;
    }
    else if (!imageQueue.empty() && !IMUQueue.empty() && AttitudeQueue.empty()) {
        return (IMUQueue.front().stamp <= imageQueue.front().stamp) ? MeasurementType::IMU : MeasurementType::Image;
    }

    // Otherwise, compare the stamps
    if ((imageQueue.front().stamp <= IMUQueue.front().stamp) && (imageQueue.front().stamp <= AttitudeQueue.front().stamp)) {
        return MeasurementType::Image;
    } 
    else if ((IMUQueue.front().stamp <= imageQueue.front().stamp) && (IMUQueue.front().stamp <= AttitudeQueue.front().stamp)){
        return MeasurementType::IMU;
    }
    else {
        return MeasurementType::Attitude;
    }
}

StampedImage ThreadedDataServer::getImage() {
    std::unique_lock lck(ioMutex);
    StampedImage data = imageQueue.front();
    imageQueue.pop();
    return data;
}

IMUVelocity ThreadedDataServer::getIMU() {
    std::unique_lock lck(ioMutex);
    IMUVelocity data = IMUQueue.front();
    IMUQueue.pop();
    return data;
}

StampedAttiude ThreadedDataServer::getAttitude() {
    std::unique_lock lck(ioMutex);
    StampedAttiude data = AttitudeQueue.front();
    AttitudeQueue.pop();
    return data;
}

double ThreadedDataServer::nextTime() const {
    const MeasurementType measType = nextMeasurementType();
    std::unique_lock lck(ioMutex);
    if (measType == MeasurementType::IMU) {
        return IMUQueue.front().stamp;
    }
    if (measType == MeasurementType::Image) {
        return imageQueue.front().stamp;
    }
    if (measType == MeasurementType::Attitude) {
        return AttitudeQueue.front().stamp;
    }
    return std::nan("");
}

ThreadedDataServer::ThreadedDataServer(std::unique_ptr<DatasetReaderBase>&& datasetReader)
    : DataServerBase(std::move(datasetReader)) {
    ioThread = std::thread(&ThreadedDataServer::fillQueues, this);
}

void ThreadedDataServer::fillQueues() {
    while ((!IMUDataFinished || !imageDataFinished|| !AttitudeDataFinished) && !destructorCalled) {

        while (!queuesFilled()) {
            std::unique_lock lck(ioMutex);

            // Fill up the image, IMU and attitude queues
            if (imageQueue.size() < maxImageQueueSize) {
                std::unique_ptr<StampedImage> nextImageData = datasetReaderPtr->nextImage();
                if (nextImageData) {
                    imageQueue.emplace(*nextImageData);
                } else {
                    imageDataFinished = true;
                }
            }

            if (AttitudeQueue.size() < maxAttitudeQueueSize) {
                std::unique_ptr<StampedAttiude> nextAttitudeData = datasetReaderPtr->nextAttitude();
                if (nextAttitudeData) {
                    AttitudeQueue.emplace(*nextAttitudeData);
                } else {
                    AttitudeDataFinished = true;
                }
            }

            if (IMUQueue.size() < maxIMUQueueSize) {
                std::unique_ptr<IMUVelocity> nextIMUData = datasetReaderPtr->nextIMU();
                if (nextIMUData) {
                    IMUQueue.emplace(*nextIMUData);
                } else {
                    IMUDataFinished = true;
                }
            }

            if (IMUDataFinished && imageDataFinished) {
                break;
            }
        }
        queuesReady_cv.notify_one();
    }
}

bool ThreadedDataServer::queuesFilled() const {
    std::unique_lock lck(ioMutex);
    return (IMUDataFinished || IMUQueue.size() == maxIMUQueueSize) &&
           (imageDataFinished || imageQueue.size() == maxImageQueueSize) &&
           (AttitudeDataFinished || AttitudeQueue.size() == maxAttitudeQueueSize);
}

ThreadedDataServer::~ThreadedDataServer() {
    destructorCalled = true;
    ioThread.join();
}