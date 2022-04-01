/**
 * @file UnicycleModel.h
 * @author Marcello Cirillo
 *
 *  Created on: Mar 29, 2011
 *      Author: marcello
 */

#ifndef UNICYCLEMODEL_H_
#define UNICYCLEMODEL_H_

#include "VehicleModel.h"
#include "WorldParameters.h"

/**
 * @class UnicycleModel
 * Models the Unicycle basic properties
 * Note: The Unicycle has its center in its current position
 */
class UnicycleModel: public VehicleModel {

protected:
	/**
	 * Implementation of the virtual function. Generates the file that contains the
	 * additional data of the motion primitives (length, occupancy)
	 */
	void generatePrimitiveAdditionalData();

	/**
	 * Implementation of the virtual function. Load motion primitives
	 */
	void loadPrimitiveLookupTable();

public:

	/**
	 * Constructor of a UnicycleModel
	 * The modelPrimitive source file must have normalized angles (-pi / pi).
	 * The physical width and length of the Unicycle are contained in the model primitives file
	 * @param modelPrimitivesFilename The name of the file of the motion primitives, without path!
	 */
	UnicycleModel(std::string modelPrimitivesFilename);

	virtual ~UnicycleModel();

	/**
	 * Returns a vectors of pointers to the cells occupied by a unicycle in
	 * a specific point.
	 * @param p A pointer to the point where the vehicle is positioned
	 * @return The vector of pointers of cells occupied
	 */
	virtual std::vector<cellPosition*> getCellsOccupiedInPosition(vehicleSimplePoint* p);
    
    /**
     * Get the vertices of the robot footprint in a given pose
     * @param p A pointer to the point where the vehicle is positioned.
     * @param b_l A container for the bottom left vertex
     * @param b_r A container for the bottom right vertex
     * @param t_l A container for the top left vertex
     * @param t_r A container for the top right vertex
     */
    virtual void getFootprintInPose(simplePoint & b_l, simplePoint & b_r, simplePoint & t_l, simplePoint & t_r, const vehicleSimplePoint* p);

};

#endif /* UNICYCLEMODEL_H_ */
