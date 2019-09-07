#---------------------------------------------------------------------------------------------------------------------
#  HECATONQUIROS
#---------------------------------------------------------------------------------------------------------------------
#  Copyright 2018 ViGUS University of Seville
#---------------------------------------------------------------------------------------------------------------------
#  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
#  and associated documentation files (the "Software"), to deal in the Software without restriction, 
#  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
#  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all copies or substantial 
#  portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
#  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
#  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
#  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
#  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#---------------------------------------------------------------------------------------------------------------------

 
### Base backend class for managing different manipulators in the library
class Backend:
    ### \brief abstract method for sending arm to a desired pose
    def pose(self, _pose, _blocking = False):
        raise NotImplementedError( "This Backend does not implements set pose method" )
        return False

    ### \brief abstract method for moving joints of the arm to the desired angle
    def joints(self, _joints, _blocking = False):
        raise NotImplementedError( "This Backend does not implements set joints method" )
        return False

    ### \brief Method to get n first joints of arm with blocking 666 Improve method.
    def joints(self, _nJoints, _blocking = False):
        raise NotImplementedError( "This Backend does not implements get joints method" )
        return []

    ### \brief abstract method for actuating to claws if implemented and attached
    ### \param _action: 0 close, 1 stop, 2 open;
    def claw(self, _action, bool _blocking = true):
        raise NotImplementedError( "This Backend does not implements set clar method" )
        return False

    ### \brief abstract method for read position of a servo
    ### \param _id
    def jointPos(self, _id):
        raise NotImplementedError( "This Backend does not implements get joint pose method" )
        return -1

    ### \brief abstract method for read Load of a servo
    ### \param _id
    def jointLoad(self, _id):
        raise NotImplementedError( "This Backend does not implements get joint load method" )
        return -1

    ### \brief abstract method for enable/disable servo torque
    ### \param _id
    ### \param _enable
    def jointTorque(self, _id, _enable):
        raise NotImplementedError( "This Backend does not implements enabling/disabling torque method" )
        return -1
    