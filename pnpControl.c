/*
 *
 * pnpControl.c - the controller for the pick and place machine in manual and autonomous mode
 *
 * Platform: Any POSIX compliant platform
 * Intended for and tested on: Cygwin 64 bit
 *
 */

#include "pnpControl.h"

// state names and numbers
#define HOME                0
#define MOVE_TO_FEEDER      1
#define WAIT              	2
#define LOWER_NOZZLE       	3
#define PICK_COMPONENT     	4
#define RAISE_COMPONENT    	5
#define MOVE_TO_CAMERA    	6
#define TAKE_UP_PHOTO    	7
#define MOVE_TO_PCB	    	8
#define TAKE_DOWN_PHOTO    	9
#define ROTATE          	10
#define ADJUST             	11
#define LOWER_COMPONENT    	12
#define PLACE_COMPONENT    	13
#define RAISE_HEAD      	14


/* state_names of up to 19 characters (the 20th character is a null terminator), only required for display purposes */
const char state_name[15][20] = {"HOME              ",
                                "MOVE TO FEEDER		",
                                "WAIT	            ",
								"LOWER_NOZZLE       ",
								"PICK_COMPONENT     ",
								"RAISE_COMPONENT    ",
								"MOVE_TO_CAMERA     ",
								"TAKE_UP_PHOTO      ",
								"MOVE_TO_PCB	    ",
								"TAKE_DOWN_PHOTO    ",
								"ROTATE  		    ",
								"ADJUST  		    ",
								"LOWER_COMPONENT    ",
								"PLACE_COMPONENT    ",
								"RAISE_HEAD         "};

const double TAPE_FEEDER_X[NUMBER_OF_FEEDERS] = {FDR_0_X, FDR_1_X, FDR_2_X, FDR_3_X, FDR_4_X, FDR_5_X, FDR_6_X, FDR_7_X, FDR_8_X, FDR_9_X};
const double TAPE_FEEDER_Y[NUMBER_OF_FEEDERS] = {FDR_0_Y, FDR_1_Y, FDR_2_Y, FDR_3_Y, FDR_4_Y, FDR_5_Y, FDR_6_Y, FDR_7_Y, FDR_8_Y, FDR_9_Y};

const char nozzle_name[3][10] = {"left", "centre", "right"};
int main()
{
    pnpOpen();

    int operation_mode, number_of_components_to_place, res;
    PlacementInfo pi[MAX_NUMBER_OF_COMPONENTS_TO_PLACE];

    /*
     * read the centroid file to obtain the operation mode, number of components to place
     * and the placement information for those components
     */
    res = getCentroidFileContents(&operation_mode, &number_of_components_to_place, pi);

    if (res != CENTROID_FILE_PRESENT_AND_READ)
    {
        printf("Problem with centroid file, error code %d, press any key to continue\n", res);
        getchar();
        exit(res);
    }

    /* state machine code for manual control mode */
    if (operation_mode == MANUAL_CONTROL)
    {
        /* initialization of variables and controller window */
        int state = HOME, finished = FALSE, picked = FALSE, adjusted = FALSE, rotated = FALSE;
		double theta_pick_error[3]= {0, 0, 0}; //array for angle errors
		double x_preplace_error = 0; //gantry x error
		double y_preplace_error = 0; //gantry y error
		double rotateAngle; //angle needed to rotate
		double adjustX, adjustY; //X & Y adjustment needed
        char c;
        int count = 0; //setup a counter to keep track of parts
        printf("Time: %7.2f  Initial state: %.15s  Operating in manual control mode, there are %d parts to place\n\n", getSimTime(), state_name[HOME], number_of_components_to_place);
        printf("Part 0 details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
        pi[count].component_designation, pi[count].component_footprint, pi[count].component_value, pi[count].x_target, pi[count].y_target, pi[count].theta_target, pi[0].feeder);

		/* loop until user quits */
        while(!isPnPSimulationQuitFlagOn())
        {
            /* print details of part 0 */

            c = getKey();

            switch (state)
            {
                case HOME:
					if (count != number_of_components_to_place) //check if there are any components to pick
					{
						finished = FALSE;
						printf("Time: %7.2f  select tape feeder to pick from \n", getSimTime());
					}
					else
                    {
                        finished = TRUE;
						printf("Time: %7.2f  All components placed - press q to quit \n", getSimTime());
                    }

                    if (finished == FALSE && (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9'))
                    {
                        /* the expression (c - '0') obtains the integer value of the number key pressed */
                        setTargetPos(TAPE_FEEDER_X[c - '0'], TAPE_FEEDER_Y[c - '0']);
                        state = MOVE_TO_FEEDER;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %c\n", getSimTime(), state_name[state], c);
                    }
                    break;

                case MOVE_TO_FEEDER:

                    if (isSimulatorReadyForNextInstruction())
                    {
                        state = WAIT;
                        printf("Time: %7.2f  New state: %.20s  Arrived at feeder, Press 'p' to pick\n", getSimTime(), state_name[state]);
                    }
                    break;

                case WAIT:

					if (picked == FALSE && (c == 'p' || c == 'P'))
					{
						state = LOWER_NOZZLE;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Lower Nozzle \n", getSimTime(), state_name[state]);
					}
					if (picked == TRUE && (c == 'c' || c == 'C'))
					{
						setTargetPos(-100,100);
						state = MOVE_TO_CAMERA;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Move to Camera \n", getSimTime(), state_name[state]);
					}
					if (theta_pick_error[1] != 0 && (c == 'r' || c == 'R') && rotated == FALSE)
					{
						state = ROTATE;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Rotate component \n", getSimTime(), state_name[state]);
					}
					if ((x_preplace_error != 0 || y_preplace_error != 0) && (c == 'a' || c == 'A') && adjusted == FALSE)
					{
						state = ADJUST;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Adjust position of Gantry \n", getSimTime(), state_name[state]);
					}
					if ((c == 'p' || c == 'P') && picked == TRUE && rotated == TRUE && adjusted == TRUE)
					{
						state = LOWER_COMPONENT;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Lower Nozzle \n", getSimTime(), state_name[state]);
					}
					if (picked == FALSE && (c == 'h' || c == 'H'))
					{
						setTargetPos(0,0);
						if (isSimulatorReadyForNextInstruction())
						state = HOME;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Return Home \n", getSimTime(), state_name[state]);
					}
                    break;
				case LOWER_NOZZLE:

                    if (isSimulatorReadyForNextInstruction())
					{
						lowerNozzle(1);
						state = PICK_COMPONENT;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to pick Component \n", getSimTime(), state_name[state]);
					}
                    break;

				case PICK_COMPONENT:

                    if (isSimulatorReadyForNextInstruction())
					{
						applyVacuum(1);
						state = RAISE_COMPONENT;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Raise Nozzle \n", getSimTime(), state_name[state]);
					}
                    break;

				case RAISE_COMPONENT:

                    if (isSimulatorReadyForNextInstruction())
					{
						raiseNozzle(1);
						state = WAIT;
						picked = TRUE;
						printf("Time: %7.2f  New state: %.20s  Component %.2f Picked. Press 'C' to move to camera and take photo\n", getSimTime(), state_name[state], pi[count].component_value);
					}
                    break;

				case MOVE_TO_CAMERA:

                    if (isSimulatorReadyForNextInstruction())
					{
						state = TAKE_UP_PHOTO;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Take Photo from Below \n", getSimTime(), state_name[state]);
					}
                    break;

				case TAKE_UP_PHOTO:

                    if (isSimulatorReadyForNextInstruction())
					{
						takePhoto(0);
						theta_pick_error[1] = getPickErrorTheta(1);
						if (theta_pick_error[1] == 0)
                        {
                            rotated = TRUE;
                        }
						printf("Time: %7.2f  Photo taken, Rotation error = %.2f \n", getSimTime(), theta_pick_error[1]);
						setTargetPos(pi[count].x_target, pi[count].y_target);
						state = MOVE_TO_PCB;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to move to PCB position x: %.2f y: %.2f \n", getSimTime(), state_name[state], pi[count].x_target, pi[count].y_target);
					}
                    break;

				case MOVE_TO_PCB:

                    if (isSimulatorReadyForNextInstruction())
					{
						printf("Time: %7.2f  Arrived at PCB position x: %.2f y: %.2f \n", getSimTime(), pi[count].x_target, pi[count].y_target);
						state = TAKE_DOWN_PHOTO;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Take Photo from Above \n", getSimTime(), state_name[state]);
					}
                    break;

				case TAKE_DOWN_PHOTO:

                    if (isSimulatorReadyForNextInstruction())
					{
						takePhoto(1);
						x_preplace_error = getPreplaceErrorX();
						y_preplace_error = getPreplaceErrorY();
						if (x_preplace_error == 0 && y_preplace_error == 0)
                        {
                            adjusted = TRUE;
                        }
						state = WAIT;
						printf("Time: %7.2f  New state: %.20s  Photos taken, Position error = x: %.2f y: %.2f\n", getSimTime(), state_name[state], x_preplace_error, y_preplace_error);
                        if (theta_pick_error[1] != 0)
                        {
                                printf("Press 'R' to Rotate\n");
                        }
                        if (x_preplace_error != 0 || y_preplace_error != 0)
                        {
                            printf("Press 'A' to adjust gantry\n");
                        }
					}
                    break;

				case ROTATE:

                    if (isSimulatorReadyForNextInstruction())
					{

						rotateAngle =  pi[count].theta_target - theta_pick_error[1];
						rotateNozzle(1, rotateAngle);
						rotated = TRUE;
						state = WAIT;
						printf("Time: %7.2f  New state: %.20s  Component Rotated , waiting for next instruction\n", getSimTime(), state_name[state]);
					}
                    break;

				case ADJUST:

                    if (isSimulatorReadyForNextInstruction())
					{

						adjustX =  pi[count].x_target - x_preplace_error;
						adjustY =  pi[count].y_target - y_preplace_error;
						amendPos(adjustX, adjustY);
						adjusted = TRUE;
						state = WAIT;
						printf("Time: %7.2f  New state: %.20s  Gantry Adjusted , waiting for next instruction\n", getSimTime(), state_name[state]);
					}
                    break;

				case LOWER_COMPONENT:

                    if (isSimulatorReadyForNextInstruction())
					{
						lowerNozzle(1);
						state = PLACE_COMPONENT;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Place component \n", getSimTime(), state_name[state]);
					}
                    break;

				case PLACE_COMPONENT:

                    if (isSimulatorReadyForNextInstruction())
					{
						releaseVacuum(1);
						state = RAISE_HEAD;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Raise Nozzle \n", getSimTime(), state_name[state]);
					}
                    break;

				case RAISE_HEAD:

                    if (isSimulatorReadyForNextInstruction())
					{
						raiseNozzle(1);
						state = WAIT;
						picked = FALSE;
						rotated = FALSE;
						adjusted = FALSE;
						count = count + 1;
						printf("Time: %7.2f  New state: %.20s  Component %.2f Placed, waiting for next instruction\n", getSimTime(), state_name[state], pi[0].component_value);
						printf("Part 0 details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
                        pi[count].component_designation, pi[count].component_footprint, pi[count].component_value, pi[count].x_target, pi[count].y_target, pi[count].theta_target, pi[0].feeder);
					}
                    break;


            }
            sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);
        }
    }
    /* state machine code for autonomous control mode */
    else
    {


    }

    pnpClose();
    return 0;
}

