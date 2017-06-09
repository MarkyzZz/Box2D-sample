#include<SDL\SDL.h>
#include<GL\freeglut.h>
#include<Box2D\Box2D.h>
#include <Box2D\Particle\b2Particle.h>
#include <Box2D/Particle/b2ParticleGroup.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#undef main
#define M_PI 3.14159265359


/*
When using box2D functions or objects we need to convert parametre values
From Pixels to Meters since Box2D uses Meters
When using OpenGL we need to convert meters to pixels since it
Proceses data in pixels
*/
b2World* world;
const int WIDTH = 780;
const int HEIGHT = 520;
const int PRECISION = 50;
const float32 step = 30.0f;
const float32 density = 0.5f;
const float32 radius = 1.5f;
const float M2P = 30;
const float P2M = 1/M2P;

int *flag = new int(1);
int *flag2 = new int(2);

b2Body *addRect(int x,int y,float32 width,float32 height, bool dyn);
b2Body *addCircle(int x,int y,float32 r, bool dyn);


void drawCircle(b2Vec2 center, float32 radius, float angle);
void drawSquare(b2Vec2* points,b2Vec2 center, float32 angle,int r,int g, int b);
void createParticle();
void createBridge(b2Body *sta1, b2Body *sta2, int w,int h);
void init();
void display();

b2Body *addRect(int x,int y,float32 width,float32 height, bool dyn=true){
	//Create a bodydef and set properties(position,type)
	b2BodyDef bodyDef;
	if(dyn)
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(P2M*x,P2M*y);
	
	//Provide a shape for a fixture
	b2PolygonShape shape;
	shape.SetAsBox(P2M*width/2,P2M*height/2);

	//Create fixture definition
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = density;
	fixtureDef.friction = 0.1f;
	fixtureDef.restitution = 0.52f;
	
	//Create a body in the world
	b2Body* body = world->CreateBody(&bodyDef);
	//Assign body a fixture
	body->CreateFixture(&fixtureDef);
	
	return body;
}

b2Body *addCircle(int x,int y,float32 r, bool dyn=true){
	//Create a bodydef and set properties(position,type)
	b2BodyDef bodyDef;
	if(dyn)
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(P2M*x,P2M*y);
	
	//Provide a shape for a fixture
	b2CircleShape shape;
	shape.m_radius = r;
	shape.m_p.Set(0,0);
	

	//Create fixture definition
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &shape;
	fixtureDef.density = 0.3f; 
	fixtureDef.friction = 1.5f;
	fixtureDef.restitution = 25*P2M;
	
	//Create a body in the world
	b2Body* body = world->CreateBody(&bodyDef);
	//Assign body a fixture
	body->CreateFixture(&fixtureDef);
	
	return body;
}

void drawCircle(b2Vec2 center, float32 radius, float angle){
	
	glColor3ub(241, 141, 158);
	glPushMatrix();
	glTranslatef(center.x*M2P,center.y*M2P,0);
	glRotatef(angle*180/M_PI,0,0,1);
	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(0,0);
	for(float i =0;i<=360;i+=360/PRECISION){
		if(i<50)
			glColor3ub(230, 215, 42);
		else if(i>=50&&i<=100)
			glColor3ub(91, 200, 172);
		else
			glColor3ub(239, 74, 102);
		glVertex2f((cos(i*M_PI/180)*radius)*M2P,(sin(i*M_PI/180)*radius)*M2P);
	}
	glEnd();
	glPopMatrix();

}

void drawSquare(b2Vec2* points,b2Vec2 center, float32 angle,int r,int g, int b){
	glColor3ub(r, g, b);
	
	glPushMatrix();
	glTranslatef(center.x*M2P,center.y*M2P,0);
	glRotatef(180*angle/M_PI,0,0,1);
	glBegin(GL_QUADS);
	for(int i =0;i<4;i++){
		glVertex2f(points[i].x*M2P,points[i].y*M2P);
	}
	glEnd();
	glPopMatrix();
}

void createBridge(b2Body* sta1,b2Body* sta2,int w,int h)
{
	b2Vec2 pos1=M2P*(sta1->GetWorldCenter()+(((b2PolygonShape*)sta1->GetFixtureList()->GetShape())->GetVertex(1)));
    b2Vec2 pos2=M2P*(sta2->GetWorldCenter()+(((b2PolygonShape*)sta2->GetFixtureList()->GetShape())->GetVertex(0)));

	sta1->SetUserData(flag2);
	sta2->SetUserData(flag2);
	

        int num=(pos2.x-pos1.x)/w;
        b2RevoluteJointDef jointDef;
        jointDef.bodyA=sta1;
        b2Body* prev,*cur;
        prev=addRect(pos1.x+(w/2),pos1.y,w,h,true);
		prev->SetUserData(flag);
        jointDef.bodyB=prev;
        jointDef.localAnchorA.Set(((b2PolygonShape*)sta1->GetFixtureList()->GetShape())->GetVertex(1).x,0);
        jointDef.localAnchorB.Set(-w/2*P2M,0);
        world->CreateJoint(&jointDef);
        for(int i=0;i<num-1;i++)
        {
                cur=addRect(pos1.x+i*w,pos1.y,w,h,true);
				cur->SetUserData(flag);
                jointDef.bodyA=prev;
                jointDef.bodyB=cur;
                jointDef.localAnchorA.Set(w/2*P2M,0);
                jointDef.localAnchorB.Set(-w/2*P2M,0);
                world->CreateJoint(&jointDef);
                prev=cur;      
        }
        jointDef.bodyA=prev;
        jointDef.bodyB=sta2;
        jointDef.localAnchorA.Set(w/2*P2M,0);
        jointDef.localAnchorB.Set((((b2PolygonShape*)sta2->GetFixtureList()->GetShape())->GetVertex(0)).x,0);
        world->CreateJoint(&jointDef); 
}

void init(){
	glMatrixMode(GL_PROJECTION);
	glOrtho(0,WIDTH,HEIGHT,0,-1,1);
	glMatrixMode(GL_MODELVIEW);
	glClearColor(123/255.0, 127/255.0, 127/255.0,1);
	
	world = new b2World(b2Vec2(0.0f*P2M,9.81f*P2M));

	addRect(WIDTH/2,HEIGHT-50,WIDTH,30,false);

	b2Body* sta1=addRect(100,150,50,10,false);
    b2Body* sta2=addRect(600,150,50,10,false);
    createBridge(sta1,sta2,50,10);
}

void display(){
	
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	b2Body *tmp = world->GetBodyList();

	b2Vec2 points[4];	
	while(tmp){
		
		if(tmp->GetFixtureList()->GetShape()->GetType()==0){
		b2CircleShape* circle = ((b2CircleShape *)tmp->GetFixtureList()->GetShape());
		drawCircle(tmp->GetWorldCenter(),radius,tmp->GetAngle());

		}else if(tmp->GetUserData()==flag){
			for(int i=0;i<4;i++)
			points[i] = ((b2PolygonShape *)tmp->GetFixtureList()->GetShape())->GetVertex(i);		
			drawSquare(points,tmp->GetWorldCenter(),tmp->GetAngle(),128, 189, 158);

		}else if(tmp->GetUserData()==flag2){
			for(int i=0;i<4;i++)
			points[i] = ((b2PolygonShape *)tmp->GetFixtureList()->GetShape())->GetVertex(i);		
			drawSquare(points,tmp->GetWorldCenter(),tmp->GetAngle(),255, 66, 14);
		}
		else{
		for(int i=0;i<4;i++)
			points[i] = ((b2PolygonShape *)tmp->GetFixtureList()->GetShape())->GetVertex(i);
			drawSquare(points,tmp->GetWorldCenter(),tmp->GetAngle(),71, 255, 245);
		} 
		tmp = tmp->GetNext();
	}

}

int main(){
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_SetVideoMode(WIDTH,HEIGHT,32,SDL_OPENGL);
	
	Uint32 start;
	
	SDL_Event event;
	bool running = true;
	bool *created = new bool[2];
	created[1] = false;
	created[0] = false;

	 b2Body *circle;
	 b2Body *rectangle;
	

	init();

	while(running){
		start = SDL_GetTicks(); // counts the number of miliseconds since the library started
		while(SDL_PollEvent(&event)){
			switch(event.type){
			case SDL_QUIT:
				running = false;
				break;

			case SDL_KEYDOWN:
				switch(event.key.keysym.sym){
				case SDLK_ESCAPE:
					running = false;
					break;
				case SDLK_RSHIFT:
				if(created[1]==true)
					circle->ApplyTorque(20*M2P,true);
					break;
				case SDLK_LSHIFT:
				if(created[1]==true)
					circle->ApplyTorque(-20*M2P,true);
					break;
				case SDLK_SPACE:
					if(created[0]==true)
						rectangle->ApplyForce(b2Vec2(0.0f,-1.0f*M2P),rectangle->GetWorldCenter(),true);
					break;
				}
				break;
			case SDL_MOUSEBUTTONDOWN:
				switch(event.button.button){
				case SDL_BUTTON_LEFT:
					circle = addCircle(event.button.x,event.button.y,radius,true);
					
					created[1] = true;
					break;

				case SDL_BUTTON_RIGHT:
					rectangle = addRect(event.button.x,event.button.y,20.0f,30.0f,true);
					created[0]=true;
					break;
				}
				break;
			}
		}
		display();
		world->Step(1.0/step,4,10); //Higher values = higher results, but lower performance 
		SDL_GL_SwapBuffers();
	}
	
	delete[] created;
	SDL_Quit();

	
}