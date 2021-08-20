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
#define COMPLETED         	15


/* state_names of up to 19 characters (the 20th character is a null terminator), only required for display purposes */
const char state_name[16][20] = {"HOME              ",
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
								"RAISE_HEAD         ",
								"COMPLETED          "};

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

    /* initialization of variables and controller window */
    int state = HOME, finished = FALSE, picked = FALSE, adjusted = FALSE, rotated = FALSE, camera = FALSE, offset = 0, i = 0;
    int autoPicked[3]={FALSE, FALSE, FALSE};
    double theta_pick_error[3]= {0, 0, 0}; //array for angle errors
    double x_preplace_error = 0; //gantry x error
	double y_preplace_error = 0; //gantry y error
	double rotateAngle; //angle needed to rotate
    char c;
    int count = 0; //setup a counter to keep track of parts
	int pickedCount = 0; //setup a counter to keep track of autoPicking
	int placedCount = 0; //setup a counter to keep track of autoPlacing

    /* state machine code for manual control mode */
    if (operation_mode == MANUAL_CONTROL)
    {

        printf("Time: %7.2f  Initial state: %.15s  Operating in manual control mode, there are %d parts to place\n\n", getSimTime(), state_name[HOME], number_of_components_to_place);
        printf("Part 0 details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
        pi[count].component_designation, pi[count].component_footprint, pi[count].component_value, pi[count].x_target, pi[count].y_target, pi[count].theta_target, pi[count].feeder);
        printf("Time: %7.2f  select tape feeder to pick from \n", getSimTime());
		/* loop until user quits */
        while(!isPnPSimulationQuitFlagOn())
        {
            /* print details of part 0 */

            c = getKey();

            switch (state)
            {

                /* Initial state - waits for correct feeder to be selected */
                case HOME:

					state = WAIT;
					if (finished == FALSE && (c - '0') == pi[count].feeder)
					                    {
                        /* the expression (c - '0') obtains the integer value of the number key pressed */
                        setTargetPos(TAPE_FEEDER_X[c - '0'], TAPE_FEEDER_Y[c - '0']);
                        state = MOVE_TO_FEEDER;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %c\n", getSimTime(), state_name[state], c);
                    }
                    else if (finished == FALSE && (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9') && (c - '0') != pi[count].feeder)
                    {
                        printf("Time: %7.2f  Feeder mismatch \n", getSimTime());
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

                    if (finished == TRUE) //check if there are any components to pick
					{
						printf("Time: %7.2f  All components placed - press q to quit \n", getSimTime());
						state = COMPLETED;
					}

					if (finished == FALSE && (c - '0') == pi[count].feeder)
					                    {
                        /* the expression (c - '0') obtains the integer value of the number key pressed */
                        setTargetPos(TAPE_FEEDER_X[c - '0'], TAPE_FEEDER_Y[c - '0']);
                        state = MOVE_TO_FEEDER;
                        printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %c\n", getSimTime(), state_name[state], c);
                    }
                    else if (finished == FALSE && (c == '0' || c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6' || c == '7' || c == '8' || c == '9') && (c - '0') != pi[count].feeder)
                    {
                        printf("Time: %7.2f  Feeder mismatch \n", getSimTime());
                    }

					if (picked == FALSE && (c == 'p' || c == 'P'))
					{
						state = LOWER_NOZZLE;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Lower Nozzle \n", getSimTime(), state_name[state]);
					}
					if (picked == TRUE && (c == 'c' || c == 'C')&& rotated == FALSE && camera == FALSE && adjusted == FALSE)
					{
						setTargetPos(-100,100);
						state = MOVE_TO_CAMERA;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Move to Camera \n", getSimTime(), state_name[state]);
					}

                    /* Rotate state - needs part picked and there to be an error after an up pic has been taken */
					if (theta_pick_error[1] != 0 && (c == 'r' || c == 'R') && rotated == FALSE && picked == TRUE && camera == TRUE)
					{
						state = ROTATE;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Rotate component \n", getSimTime(), state_name[state]);
					}

					/* Adjust state - needs part picked and there to be an error after a down pic has been taken */
					if ((x_preplace_error != 0 || y_preplace_error != 0) && (c == 'a' || c == 'A') && adjusted == FALSE && picked == TRUE && camera == TRUE)
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
                    camera  = TRUE;
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
						sleepMilliseconds(1000);
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
						sleepMilliseconds(1000);
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
                        amendPos(x_preplace_error, y_preplace_error);
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
						//Reset variables
						state = WAIT;
						picked = FALSE;
						rotated = FALSE;
						adjusted = FALSE;
						camera = FALSE;
						//increase counter
						count = count + 1;
						printf("Time: %7.2f  New state: %.20s  Component %.2f Placed, waiting for next instruction\n", getSimTime(), state_name[state], pi[count].component_value);

						if (count == number_of_components_to_place) //check if there are any components to pick
                        {
                            finished = TRUE;
                        }
                        else if (count != number_of_components_to_place)
                        {
                            printf("Part details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
                            pi[count].component_designation, pi[count].component_footprint, pi[count].component_value, pi[count].x_target, pi[count].y_target, pi[count].theta_target, pi[count].feeder);
                            printf("Time: %7.2f  select tape feeder to pick from \n", getSimTime());
                        }

					}
                    break;

                    case COMPLETED:

                    if (isSimulatorReadyForNextInstruction())
					{

						while(!isPnPSimulationQuitFlagOn())
                        {
                            c = getKey();
                            if(c != '\0')
                            {
                                printf("Time: %7.2f  All components placed - press q to quit \n", getSimTime());
                            }
                        }
                        break;

					}
                    break;

            }
            sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);
        }
    }

	/* state machine code for autonomous control mode */
    //*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
	//
	//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
	//
	//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
	//
	//*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

	else
    {

        while(!isPnPSimulationQuitFlagOn())
        {
            /* print details of part 0 */

            c = getKey();

            switch (state)
            {

                /* Initial state - waits for correct feeder to be selected */
                case HOME:

                        if (pickedCount == 0)//initial loop, display components to place
                        {
                            //sort the centroid file and print it to the terminal
                            qsort (pi, number_of_components_to_place, sizeof(PlacementInfo), compare);
                            printf("Time: %7.2f  Operating in Auto control mode, there are %d parts to place\n\n", getSimTime(), number_of_components_to_place);
                            for(int k = 0; k < number_of_components_to_place; k++)
                            {
                                printf("Part %d details:\nDesignation: %s\nFootprint: %s\nValue: %.2f\nx: %.2f\ny: %.2f\ntheta: %.2f\nFeeder: %d\n\n",
                                k, pi[k].component_designation, pi[k].component_footprint, pi[k].component_value, pi[k].x_target, pi[k].y_target, pi[k].theta_target, pi[k].feeder);
                            }
                        }
                        //all components pplaced
                        if (placedCount == number_of_components_to_place)
                        {

                            state = COMPLETED;
                            break;

                        }

                        //all components picked
						if (pickedCount == number_of_components_to_place)//exit and place components when no more to pick
						{
							picked = TRUE;
						}

                        //nozzles are not full
						if (picked == FALSE)//pick 3 components
                        {
                            if(i == 0)
                            {
                                offset = 20;
                            }
                            if(i == 1)
                            {
                                offset = 0;
                            }
                            if(i == 2)
                            {
                                offset = -20;
                            }

                            setTargetPos(TAPE_FEEDER_X[pi[pickedCount].feeder]+offset, TAPE_FEEDER_Y[pi[pickedCount].feeder]);
                            printf("Time: %7.2f  New state: %.20s  Issued instruction to move to tape feeder %d\n", getSimTime(), state_name[state], pi[pickedCount].feeder);
                            state = MOVE_TO_FEEDER;
                        }

                        //all nozzles have a part or no parts left to pick
						if (picked == TRUE)
                        {
							state = ROTATE;
							if (autoPicked[0] == TRUE)
							{
								i = 0;
								break;
							}
							if (autoPicked[1] == TRUE)
							{
								i = 1;
								break;
							}
							if (autoPicked[2] == TRUE)
							{
								i = 2;
								break;
							}
						}

					break;

				case MOVE_TO_FEEDER:

					if (isSimulatorReadyForNextInstruction())
					{
						state = LOWER_NOZZLE;
						printf("Time: %7.2f  New state: %.20s  Arrived at feeder, Ready to pick\n", getSimTime(), state_name[state]);
					}
					break;

				case LOWER_NOZZLE:

                    if (isSimulatorReadyForNextInstruction())
					{
						lowerNozzle(i);
						state = PICK_COMPONENT;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to pick Component \n", getSimTime(), state_name[state]);
					}
					break;

                case PICK_COMPONENT:

                    if (isSimulatorReadyForNextInstruction())
					{
						applyVacuum(i);
						state = RAISE_COMPONENT;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Raise Nozzle \n", getSimTime(), state_name[state]);
					}
					break;

                case RAISE_COMPONENT:

                    if (isSimulatorReadyForNextInstruction())
					{
						raiseNozzle(i);
						autoPicked[i] = TRUE;
						pickedCount++;
						printf("Time: %7.2f  New state: %.20s  Component Picked \n", getSimTime(), state_name[state]);
						if (i < 2)
						{
							i++;
							state = HOME;
						}
						else if (i == 2)
						{
							i = 0;
							picked = TRUE;
							state = TAKE_UP_PHOTO;
						}
					}
					break;

				case TAKE_UP_PHOTO:

                    if (isSimulatorReadyForNextInstruction())
					{
						takePhoto(0);
						state = HOME;
						printf("Time: %7.2f  Up Photo taken\n", getSimTime());
					}
					break;

				case ROTATE:

                    if (isSimulatorReadyForNextInstruction())
					{
                        theta_pick_error[i] = getPickErrorTheta(i);
						rotateAngle =  pi[placedCount].theta_target - theta_pick_error[i];
						rotateNozzle(i, rotateAngle);
						state = MOVE_TO_PCB;
						printf("Time: %7.2f  New state: %.20s  Component Rotation = %.2f error = %.2f Total = %.2f  \n", getSimTime(), state_name[state], pi[placedCount].theta_target, theta_pick_error[i], rotateAngle);
					}
					break;

				case MOVE_TO_PCB:

                    if (isSimulatorReadyForNextInstruction())
					{
						setTargetPos(pi[placedCount].x_target, pi[placedCount].y_target);
						state = TAKE_DOWN_PHOTO;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Take Photo from Above \n", getSimTime(), state_name[state]);
					}
					break;

				case TAKE_DOWN_PHOTO:

                    if (isSimulatorReadyForNextInstruction())
					{
						takePhoto(1);
						state = ADJUST;
						printf("Time: %7.2f  New state: %.20s  Photos taken\n", getSimTime(), state_name[state]);

					}
					break;

				case ADJUST:

                    if (isSimulatorReadyForNextInstruction())
					{
						amendPos(x_preplace_error, y_preplace_error);
						state = LOWER_COMPONENT;
						printf("Time: %7.2f  New state: %.20s  Gantry Adjusted, Position error = x: %.2f y: %.2f\n", getSimTime(), state_name[state], x_preplace_error, y_preplace_error);
					}
                    break;

				case LOWER_COMPONENT:

                    if (isSimulatorReadyForNextInstruction())
					{
						lowerNozzle(i);
						state = PLACE_COMPONENT;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Place component \n", getSimTime(), state_name[state]);
					}
                    break;

				case PLACE_COMPONENT:

                    if (isSimulatorReadyForNextInstruction())
					{
						releaseVacuum(i);
						state = RAISE_HEAD;
						printf("Time: %7.2f  New state: %.20s  Issued instruction to Raise Nozzle \n", getSimTime(), state_name[state]);
					}
                    break;

				case RAISE_HEAD:

                    if (isSimulatorReadyForNextInstruction())
					{
						raiseNozzle(i);
						//increase counter
						placedCount++;
						printf("Time: %7.2f  New state: %.20s  Component %d Placed, waiting for next instruction\n", getSimTime(), state_name[state], placedCount);

						//Reset variables
						state = HOME;
						autoPicked[i] = FALSE;

						if (i == 2)
						{
							//all nozzles empty, reset flag and counter
							picked = FALSE;
							i = 0;
						}

						if (placedCount == number_of_components_to_place) //check if there are any components to pick
                        {
                            state = COMPLETED;
                            break;
                        }

					}
                    break;

                case COMPLETED:

                    if (isSimulatorReadyForNextInstruction())
                    {
                        setTargetPos(0,0);
                        printf("All components places - hit q to quit\n");
                        while(!isPnPSimulationQuitFlagOn())
                        {
                            c = getKey();
                            if(c != '\0')
                            {
                                printf("Time: %7.2f  All components placed - press q to quit \n", getSimTime());
                            }
                        }
					}
					break;

            }
            sleepMilliseconds((long) 1000 / POLL_LOOP_RATE);
        }


    }

    pnpClose();
    return 0;
}
