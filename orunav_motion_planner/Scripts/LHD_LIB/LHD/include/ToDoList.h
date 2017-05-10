// Time-stamp: <2012-06-02 12:51:48 (drdv)>

/** \page mytodo The TODO list
    
    \mytodo Each path segment is scaled to have length equal to 1 (independently of the others), so
    we can not compare the segments directly. For that reason, when we assign rate of change of arc
    length (in poly3_get_coefficients(...)) we have to be careful as velocity = 0.05 in segment 1
    can have completely different meaning from velocity = 0.05 for segment 2. I have to find the
    actual arc length of all segments and work with it. Additional problem is that the time length
    for traversing the different segments can be different and this has to be accounted for when
    choosing the initial and final velocity at via points.

*/
