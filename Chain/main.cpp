//Author: Dr. Muhammad Mobeen Movania
//Last Modified: 1 April 2014 13:15 PST

#include <GL/freeglut.h> 
#include <iostream>

//Havok headers

// Keycode
#include <Common/Base/keycode.cxx> 
#include <Common/Base/Config/hkProductFeatures.cxx>

// Math and base includes
#include <Common/Base/hkBase.h>
#include <Common/Base/System/hkBaseSystem.h>
#include <Common/Base/System/Error/hkDefaultError.h>
#include <Common/Base/Memory/System/hkMemorySystem.h>
#include <Common/Base/Memory/System/Util/hkMemoryInitUtil.h>
#include <Common/Base/Memory/Allocator/Malloc/hkMallocAllocator.h>
#include <Common/Base/Thread/Job/ThreadPool/Cpu/hkCpuJobThreadPool.h>

#include <Physics2012/Dynamics/World/hkpWorld.h>
#include <Physics2012/Collide/Dispatch/hkpAgentRegisterUtil.h>

// Visual Debugger includes
#include <Common/Visualize/hkVisualDebugger.h>
#include <Physics2012/Utilities/VisualDebugger/hkpPhysicsContext.h>				

//add headers for box shape
#include <Physics2012\Collide\Shape\Convex\Box\hkpBoxShape.h>
#include <Physics2012\Dynamics\Entity\hkpRigidBody.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>
#include <Physics2012\Dynamics\World\hkpSimulationIsland.h>
 
#include <vector>

// We need to create a constraint
#include <Physics/Constraint/Data/StiffSpring/hkpStiffSpringConstraintData.h>
//raycast objects
#include <Physics2012\Collide\Query\CastUtil\hkpWorldRayCastInput.h>
#include <Physics2012\Collide\Query\CastUtil\hkpWorldRayCastOutput.h>
#include <Physics2012\Collide/Query/Multithreaded/RayCastQuery/hkpRayCastQueryJobs.h> 

 
//for raycasthitcollector
#include <Physics2012/Collide/Query/Collector/RayCollector/hkpClosestRayHitCollector.h>
//Add linker libraries
#pragma comment(lib, "hkBase.lib") 
#pragma comment(lib, "hkCompat.lib")
#pragma comment(lib, "hkGeometryUtilities.lib")              
#pragma comment(lib, "hkInternal.lib")
#pragma comment(lib, "hkSceneData.lib")                                 
#pragma comment(lib, "hkSerialize.lib")
#pragma comment(lib, "hkVisualize.lib")
#pragma comment(lib, "hkaInternal.lib")
#pragma comment(lib, "hkaAnimation.lib")
#pragma comment(lib, "hkaPhysics2012Bridge.lib") 
#pragma comment(lib, "hkpConstraint.lib")
#pragma comment(lib, "hkpConstraintSolver.lib")
#pragma comment(lib, "hkpCollide.lib")                          
#pragma comment(lib, "hkpDynamics.lib")
#pragma comment(lib, "hkpInternal.lib")      
#pragma comment(lib, "hkpUtilities.lib")
#pragma comment(lib, "hkpVehicle.lib")  
#pragma comment(lib, "hkcdCollide.lib")
#pragma comment(lib, "hkcdInternal.lib")

 
const int	WINDOW_WIDTH=800, 
WINDOW_HEIGHT=600;

 
//for mouse dragging
int oldX=0, oldY=0;
float rX=15, rY=-45;
float fps=0;
int startTime=0;
int totalFrames=0;
int state =1 ;
float tx=0, ty=0, tz=-25;
 
//Havok variables
hkpPhysicsContext* g_pContext;
hkVisualDebugger* g_pVdb;

hkJobQueue* g_pJobQueue;
hkJobThreadPool* g_pThreadPool;

hkpWorld* g_pWorld;			// Physics world.
hkpWorldCinfo g_pWorldInfo; // Info about global simulation parameters.

bool g_bVdbEnabled = true;
static const int numStepsPerSecond = 60;
static const hkReal timeStep = 1.0f / numStepsPerSecond;

const int MAX_BOXES = 10;
std::vector<hkpRigidBody*> boxes;

bool wasHit =false;
float hitz = 0;
 
hkpRigidBody* gSelectedActor = 0; 
GLdouble MV[16];
GLdouble P[16];
GLint viewport[4];

void ViewProject(hkVector4 &v, int &xi, int &yi, float &depth)
{  	  
	GLdouble winX, winY, winZ;

	gluProject(	(GLdouble) v.getComponent(0), 
				(GLdouble) v.getComponent(1), 
				(GLdouble) v.getComponent(2), 
				MV, 
				P, 
				viewport, 
				&winX, 
				&winY, 
				&winZ);
	xi = (int)winX; 
	yi = viewport[3] - (int)winY - 1; 
	depth = (float)winZ; 
}

// ------------------------------------------------------------------------------------

void ViewUnProject(int xi, int yi, float depth, hkVector4 &v)
{
 	yi = viewport[3] - yi - 1;
	GLdouble wx=0, wy=0, wz=0;
	gluUnProject((GLdouble) xi, (GLdouble) yi, (GLdouble) depth,
	MV, P, viewport, &wx, &wy, &wz);
	v.set( wx, wy, wz, 1);
}

void SetOrthoForFont()
{	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
	glScalef(1, -1, 1);
	glTranslatef(0, -WINDOW_HEIGHT, 0);
	glMatrixMode(GL_MODELVIEW);	
	glLoadIdentity();
}

void ResetPerspectiveProjection() 
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void RenderSpacedBitmapString(
							  int x, 
							  int y,
							  int spacing, 
							  void *font,
							  char *string) 
{
	char *c;
	int x1=x;
	for (c=string; *c != '\0'; c++) {
		glRasterPos2i(x1,y);
		glutBitmapCharacter(font, *c);
		x1 = x1 + glutBitmapWidth(font,*c) + spacing;
	}
}


void DrawAxes()
{	 
	//To prevent the view from disturbed on repaint
	//this push matrix call stores the current matrix state
	//and restores it once we are done with the arrow rendering
	glPushMatrix();
		glColor3f(0,0,1);
		glPushMatrix();
			glTranslatef(0,0, 0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label			
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "Z");
		glPopMatrix();					
		glutSolidCone(0.0225,1, 4,1);

		glColor3f(1,0,0);
		glRotatef(90,0,1,0);	
		glPushMatrix();
			glTranslatef(0,0,0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "X");
		glPopMatrix();					
		glutSolidCone(0.0225,1, 4,1);

		glColor3f(0,1,0);
		glRotatef(90,-1,0,0);	
		glPushMatrix();
			glTranslatef(0,0, 0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "Y");
		glPopMatrix();					
		glutSolidCone(0.0225,1, 4,1);	
	glPopMatrix();
}
void DrawGrid(int GRID_SIZE)
{
	glBegin(GL_LINES);
	glColor3f(0.75f, 0.75f, 0.75f);
	for(int i=-GRID_SIZE;i<=GRID_SIZE;i++)
	{
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);

		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}

void OnError(const char* msg, void* userArgGivenToInit) {
	std::cerr << "Report: " << msg << std::endl; 
}

void InitMemory() {
	 #if defined(HK_COMPILER_HAS_INTRINSICS_IA32) && HK_CONFIG_SIMD == HK_CONFIG_SIMD_ENABLED
	// Flush all denormal/subnormal numbers (2^-1074 to 2^-1022) to zero.
	// Typically operations on denormals are very slow, up to 100 times slower than normal numbers.
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	#endif 

	// Initialize the base system including our memory system
	// Allocate 0.5MB of physics solver buffer.
	hkMemoryRouter* memoryRouter = hkMemoryInitUtil::initDefault( hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo( 500* 1024 ) );
	
	hkBaseSystem::init( memoryRouter, OnError );


	// We can cap the number of threads used - here we use the maximum for whatever multithreaded platform we are running on. This variable is
	// set in the following code sections.
	int totalNumThreadsUsed;

	// Get the number of physical threads available on the system
	hkHardwareInfo hwInfo;
	hkGetHardwareInfo(hwInfo);
	totalNumThreadsUsed = hwInfo.m_numThreads;

	// We use one less than this for our thread pool, because we must also use this thread for our simulation
	hkCpuJobThreadPoolCinfo threadPoolCinfo;
	threadPoolCinfo.m_numThreads = totalNumThreadsUsed - 1;

	// This line enables timers collection, by allocating 200 Kb per thread.  If you leave this at its default (0),
	// timer collection will not be enabled.
	threadPoolCinfo.m_timerBufferPerThreadAllocation = 200000;
	g_pThreadPool = new hkCpuJobThreadPool( threadPoolCinfo );

	// We also need to create a Job queue. This job queue will be used by all Havok modules to run multithreaded work.
	// Here we only use it for physics.
	hkJobQueueCinfo info;
	info.m_jobQueueHwSetup.m_numCpuThreads = totalNumThreadsUsed;
	g_pJobQueue = new hkJobQueue(info);

	// Enable monitors for this thread.
	// Monitors have been enabled for thread pool threads already (see above comment).
	hkMonitorStream::getInstance().resize(200000);
}

void InitPhysicalWorld() { 

	// Set the simulation type of the world to multi-threaded.
	g_pWorldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_MULTITHREADED;

	// Flag objects that fall "out of the world" to be automatically removed.
	g_pWorldInfo.m_broadPhaseBorderBehaviour = hkpWorldCinfo::BROADPHASE_BORDER_DO_NOTHING;
	 
	g_pWorld = new hkpWorld(g_pWorldInfo);
	
	// Disable deactivation, so that you can view timers in the VDB. This should not be done in your game.
	g_pWorld->m_wantDeactivation = false;

	// When the simulation type is SIMULATION_TYPE_MULTITHREADED, in the debug build, the sdk performs checks
	// to make sure only one thread is modifying the world at once to prevent multithreaded bugs. Each thread
	// must call markForRead / markForWrite before it modifies the world to enable these checks.
	g_pWorld->markForWrite();


	// Register all collision agents.
	// It's important to register collision agents before adding any entities to the world.
	hkpAgentRegisterUtil::registerAllAgents( g_pWorld->getCollisionDispatcher() );

	// We need to register all modules we will be running multi-threaded with the job queue.
	g_pWorld->registerWithJobQueue( g_pJobQueue );

}

void InitVDB() {
	hkArray<hkProcessContext*> contexts;

	// <PHYSICS-ONLY>: Register physics specific visual debugger processes
	// By default the VDB will show debug points and lines, however some products such as physics and cloth have additional viewers
	// that can show geometries etc and can be enabled and disabled by the VDB app.
	{
		// Initialise the visual debugger so we can connect remotely to the simulation.
		// The context must exist beyond the use of the VDB instance, and you can make
		// whatever contexts you like for your own viewer types.
		g_pContext = new hkpPhysicsContext();
		hkpPhysicsContext::registerAllPhysicsProcesses(); // all the physics viewers
		g_pContext->addWorld(g_pWorld); // add the physics world so the viewers can see it
		contexts.pushBack(g_pContext);

		// Now we have finished modifying the world, release our write marker.
		g_pWorld->unmarkForWrite();
	}

	g_pVdb = new hkVisualDebugger(contexts);
	g_pVdb->serve();
}

void ShutdownVDB() { 
	g_pVdb->removeReference();

	// Contexts are not reference counted at the base class level by the VDB as
	// they are just interfaces really. So only delete the context after you have
	// finished using the VDB.
	g_pContext->removeReference();
}

void AddRigidBodies() {
	//add the falling box 
	{
		hkVector4 halfExtents(0.5f, 0.5f, 0.125f);
		hkpBoxShape* boxShape = new hkpBoxShape(halfExtents);
		boxShape->setRadius(0.001f);

		hkpRigidBodyCinfo ci;
		ci.m_shape = boxShape;
		ci.m_motionType = hkpMotion::MOTION_DYNAMIC;
		const hkReal boxMass(10.0f);
		hkMassProperties massProps;
		hkpInertiaTensorComputer::computeShapeVolumeMassProperties(boxShape, boxMass, massProps);
		ci.setMassProperties(massProps);
				
		for(int i=0;i<MAX_BOXES;++i) {
			if(i==(MAX_BOXES-1))
				ci.m_motionType = hkpMotion::MOTION_FIXED;
			ci.m_position = hkVector4(0.0f,1.0f+i*1.125f,0.0f);	
			hkpRigidBody* rigidBody = new hkpRigidBody(ci);			
			boxes.push_back(static_cast<hkpRigidBody*>(g_pWorld->addEntity(rigidBody))); 
		}		
		boxShape->removeReference();

		//now add springs between adjacent boxes
		hkpStiffSpringConstraintData* spring = new hkpStiffSpringConstraintData(); 
		hkpStiffSpringConstraintData* spring2 = new hkpStiffSpringConstraintData(); 

		for(size_t i=0;i<boxes.size()-1;i++) {
			hkTransform b1 = boxes[i]->getTransform();
			hkTransform b2 = boxes[i+1]->getTransform();
			hkVector4f t1 = b1.getTranslation(); 
			hkVector4f t2 = b2.getTranslation();
			t1.add(hkVector4f(-0.5,0.5,0));
			t2.add(hkVector4f(-0.5,-0.5,0));

			spring->setInWorldSpace(b1,b2, t1, t2);
			{
				hkpConstraintInstance* constraint = new hkpConstraintInstance(boxes[i], boxes[i+1], spring );
					g_pWorld->addConstraint(constraint);
				constraint->removeReference(); 
			}


			t1.add(hkVector4f(1,0,0));
			t2.add(hkVector4f(1,0,0));

			spring2->setInWorldSpace(b1,b2, t1, t2);
			{
				hkpConstraintInstance* constraint = new hkpConstraintInstance(boxes[i], boxes[i+1], spring2 );
					g_pWorld->addConstraint(constraint);
				constraint->removeReference(); 
			}
		}
		spring->removeReference();
		spring2->removeReference(); 
	}

	//create the static box where the smaller box will fall
	{
		hkVector4 halfExtents(20.0f, 2.0f, 20.f);
		hkpBoxShape* boxShape = new hkpBoxShape(halfExtents);
		hkpRigidBodyCinfo ci;
		ci.m_shape = boxShape;
		ci.m_position = hkVector4(0, -2, 0);
		ci.m_motionType = hkpMotion::MOTION_FIXED;
		boxShape->setRadius(0.001f);
		hkpRigidBody* rigidBody = new hkpRigidBody(ci);
		boxShape->removeReference();
		g_pWorld->addEntity(rigidBody)->removeReference();
	}
}


void InitializeHavok() { 

	InitMemory();
	InitPhysicalWorld();
	
	if (g_bVdbEnabled)
		InitVDB();

	g_pWorld->lock();
		AddRigidBodies();	
	g_pWorld->unlock();
}
 
void DrawBox(hkpRigidBody* box) { 
	float mat[16]={}; 

	g_pWorld->markForRead();
	{ 
		hkMatrix4 m;
		hkTransform transform;
		box->approxCurrentTransform( transform );
		m.set( transform ); 	 
		memcpy(mat, &m, sizeof(float)*16);
	}
	g_pWorld->unmarkForRead();

	glPushMatrix(); 
		glMultMatrixf(mat);
		glScalef(1,1,0.25f);
		glutSolidCube(1);
    glPopMatrix(); 
} 

void StepVDB() {
	g_pContext->syncTimers(g_pThreadPool);
	g_pVdb->step(timeStep);
}

void StepHavok() { 
	
	g_pWorld->stepMultithreaded( g_pJobQueue, g_pThreadPool, timeStep);
	
	if(g_bVdbEnabled)
		StepVDB();

	// Clear accumulated timer data in this thread and all slave threads
	hkMonitorStream::getInstance().reset();
	g_pThreadPool->clearTimerData();
}

void ShutdownHavok() {   
	g_pWorld->markForWrite();
	for(int i=0;i<MAX_BOXES;++i) {
		boxes[i]->removeReference();
	}
	g_pWorld->unmarkForWrite();

	if(g_pWorld)
	{ 
		g_pWorld->markForWrite();		
		g_pWorld->removeReference();
		g_pWorld = HK_NULL;
	}	
	delete g_pJobQueue;

	// Clean up the thread pool
	g_pThreadPool->removeReference();
	
	if (g_bVdbEnabled)
		ShutdownVDB();
		 
	boxes.clear(); 
	 
	hkBaseSystem::quit();
    hkMemoryInitUtil::quit();
}

void InitGL() { 
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);

	GLfloat ambient[4]={0.0f,0.0f,0.0f,0.0f};
	GLfloat diffuse[4]={0.5,0.5,0.5,1}; 

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse); 
	

	glDisable(GL_LIGHTING);
	 
}

void OnReshape(int nw, int nh) {
	glViewport(0,0,nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)nw / (GLfloat)nh, 0.1f, 10000.0f); 
	glMatrixMode(GL_MODELVIEW);

	glGetDoublev(GL_PROJECTION_MATRIX, P);
	glGetIntegerv(GL_VIEWPORT, viewport);
}

char buffer[MAX_PATH];
void OnRender() {
	//Calculate fps
	totalFrames++;
	int current = glutGet(GLUT_ELAPSED_TIME);
	if((current-startTime)>1000)
	{		
		float elapsedTime = float(current-startTime);
		fps = ((totalFrames * 1000.0f)/ elapsedTime) ;
		startTime = current;
		totalFrames=0;
	}

	sprintf_s(buffer, "FPS: %3.2f",fps);

	StepHavok(); 

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(tx,ty,tz);
	glRotatef(rX,1,0,0);
	glRotatef(rY,0,1,0);

	glGetDoublev(GL_MODELVIEW_MATRIX, MV);

	//Draw the grid and axes
	DrawAxes();	
	DrawGrid(10);
	
	glEnable(GL_LIGHTING);  
		for(int i=0;i<MAX_BOXES;++i) {
			if(wasHit && boxes[i] == gSelectedActor)
				glColor3f(0,1,0);
			else
				glColor3f(1,1,1);
			DrawBox(boxes[i]);
		}
	glDisable(GL_LIGHTING);

	SetOrthoForFont();		
		glColor3f(1,1,1);
		//Show the fps
		RenderSpacedBitmapString(20,20,0,GLUT_BITMAP_HELVETICA_12,buffer);

	ResetPerspectiveProjection();

	glutSwapBuffers();
}

void OnShutdown() {
	ShutdownHavok();
}


void PickActor(int x, int y) { 

	hkVector4f rayStart(0,0,0,0);
	hkVector4f rayEnd(0,0,0,0);

	ViewUnProject(x, y, 0.0f, rayStart);
	ViewUnProject(x, y, 1.0f, rayEnd);
	 
	hkpWorldRayCastInput input;
	hkpClosestRayHitCollector result; 

	input.m_from = rayStart;
	input.m_to = rayEnd;
	input.m_filterInfo = 0;

	g_pWorld->lock();
	g_pWorld->castRay(input, result);
	g_pWorld->unlock();
	
	 
	if(result.hasHit()) {
		hkpRigidBody* pRB = hkpGetRigidBody(result.getHit().m_rootCollidable);
		wasHit = pRB->getMotionType() != hkpMotion::MOTION_FIXED;
		if(wasHit) {
			gSelectedActor = pRB;
			hkVector4f pos2 = gSelectedActor->getTransform().getTranslation();
			hkVector4f intersectionPointWorld;
			intersectionPointWorld.setInterpolate4( input.m_from, input.m_to, result.getHit().m_hitFraction );

			int hitx=0, hity=0;
			ViewProject(intersectionPointWorld, hitx, hity, hitz); 

			g_pWorld->lock();
				gSelectedActor->setMotionType(hkpMotion::MOTION_FIXED);
			g_pWorld->unlock();
		}  
	} else {
		wasHit = false;
	} 
}

void MoveActor(int x, int y) {
	
	if (!wasHit) 
		return;
	
	hkVector4 pos;
	ViewUnProject(x,y, hitz, pos); 

	g_pWorld->lock();		  
		hkTransform trans = gSelectedActor->getTransform(); 
		trans.setTranslation(pos); 
		gSelectedActor->setTransform(trans);
	g_pWorld->unlock();	  
	
}

void Mouse(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN) 
	{
		if(button== GLUT_LEFT_BUTTON) {
			PickActor(x,y); 
		} 
		
		if(!wasHit) {
			if(button == GLUT_MIDDLE_BUTTON)
				state = 0;
			else if(button == GLUT_RIGHT_BUTTON)
				state = 2;
			else
				state = 1;
		}

		oldX=x;
		oldY=y;
	}	
	
	if (s == GLUT_UP ) {
		
		if(wasHit) {
			g_pWorld->lock();
				gSelectedActor->setMotionType(hkpMotion::MOTION_DYNAMIC);
			g_pWorld->unlock();
		}
		wasHit = false;
		state=-1;
	}
}

void Motion(int x, int y)
{
	if (wasHit)
	{
		MoveActor(x,y);
	} else {
		if (state == 0)
			tz *= (1 + (y - oldY)/60.0f); 
		else if (state == 2) {
			tx -= (x - oldX)/50.0f; 
			ty += (y - oldY)/50.0f; 
		} else if (state == 1 )  {
			rY += (x - oldX)/5.0f; 
			rX += (y - oldY)/5.0f; 
		} 
	}
	
	oldX = x; 
	oldY = y; 
	glutPostRedisplay(); 
}

void OnIdle() {
	glutPostRedisplay();
}

void main(int argc, char** argv) {
	
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("GLUT Havok Demo - Chain");
	 

	glutDisplayFunc(OnRender);
	glutIdleFunc(OnIdle);
	glutReshapeFunc(OnReshape);
	glutCloseFunc(OnShutdown);

	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);
	InitGL();
	InitializeHavok();

	glutMainLoop();		
}
