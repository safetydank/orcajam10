#include "boost/foreach.hpp"
#define FOREACH BOOST_FOREACH

#include "cinder/app/AppBasic.h"
#include "cinder/ImageIO.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
#include "cinder/Camera.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/Timer.h"
#include "cinder/BSpline.h"
#include "cinder/Rand.h"
#include "portaudio.h"

// #include "cinder/Capture.h"

#define PA_SAMPLE_RATE  (44100)
#define FRAMES_PER_BUFFER (1024)
#define NUM_CHANNELS    (1)

#define PA_SAMPLE_TYPE  paFloat32
typedef float SAMPLE;
#define SAMPLE_SILENCE  (0.0f)
#define PRINTF_S_FORMAT "%.8f"	
// const int SAMPLE_FRAMES = 1024;

#include "helper.h"
#include "ParticleController.h"
#include "Config.h"
#include "Resources.h"

using namespace ci;
using namespace ci::app;

using std::vector;

#define VWIDTH 640
#define VHEIGHT 480

#define SCREEN_WIDTH  1024
#define SCREEN_HEIGHT 768

#define FRAMERATE 30.0f

//  envelope attack and release
const float attack  = 0.1;
const float release = 0.1;
float env_ga = (float) exp(-1/(PA_SAMPLE_RATE*attack));
float env_gr = (float) exp(-1/(PA_SAMPLE_RATE*release));
float audioScale = 1.0f;
float envelope = 0; 

float GSPEED = 1.0f;

// global timer
ci::Timer CTimer;

const float lowerY = -0.9f;
struct Player
{
    float radius;
	Vec2f position;
	Vec2f velocity;
    bool pushUp;

	Player() : position(Vec2f(0,0)), velocity(Vec2f(0,0)), radius(0.02f), pushUp(true)
	{
	}

    void reverseDirection() { pushUp = !pushUp; }

	void update()
	{
		const float dampen = 0.98f;
		const float dt2 = (1.0f/FRAMERATE) * (1.0f/FRAMERATE);

		float accy = (envelope*0.8f) - 0.01f;
        if (!pushUp) accy = -accy;

		velocity = Vec2f(velocity.x, math<float>::clamp((velocity.y + accy) * dampen, -0.04f, 0.04f));

		position.y += velocity.y + 0.5*accy*dt2;
		if (position.y < lowerY || position.y > 1.0f) {
			velocity = -velocity;
			// accy = 0;
		}
		position = Vec2f(math<float>::clamp(position.x, -1.0f, 1.0f), 
						 math<float>::clamp(position.y, lowerY, -lowerY));		
	}

	void draw()
	{
	}
};

struct Planet
{
	Vec2f           position;
	int             kind;
    bool            active;
    float           radius;
    bool            collided;
    BSpline<Vec2f>  path;
    float           speed;
    float           elapsed;
    bool            completed;

    bool opposed;  // DEBUG

    Planet(float radius) : position(Vec2f(0,0)), kind(0), active(false), opposed(false), radius(radius), collided(false), speed(1.0f), elapsed(0), completed(false) {  }

    void update() {
        if (active) {
            elapsed += (1.0f / FRAMERATE) * speed * GSPEED;
            console() << "Elapsed: " << elapsed << std::endl;
            if (elapsed > 1.0f) {
                //  end of the path
                stop();
                completed = true;
            }
            // // position = Vec2f(sin(elapsed), cos(elapsed));
            // position = 0.87f * (opposed ? -1.0f : 1.0f) * Vec2f(sin(elapsed), cos(elapsed));
            // // position *= opposed ? 0.87f : -0.87f;

            position = path.getPosition(elapsed);
        }
        else {
            position = Vec2f(100.0f, 100.0f);
        }
	}

    void start() {
        active = true;
        elapsed = 0;
        completed = false;
    }

    void stop() {
        active = false;
        elapsed = 0;
    }
};


//  Planet formations defined by:
//
//  A set of control points to follow for t E [0,1]
//  Starting times for each
struct Formation
{
    vector<Vec2f> lightPoints;
    vector<Vec2f> darkPoints;
    float lightSpeed;
    float darkSpeed;
    bool lightActive;
    bool darkActive;

    float lightStart;
    float darkStart;

    float elapsed;

    Planet& light;
    Planet& dark;

    bool active;

    Formation(Planet& light, Planet& dark) : lightSpeed(1.0f), lightActive(false), darkActive(false), darkSpeed(1.0f), lightStart(0), darkStart(0), light(light), dark(dark)
    { }

    void start() {
        active = true;
        light.completed = false;
        dark.completed = false;

        elapsed = 0;
        light.stop();
        dark.stop();
        if (lightActive) {
            light.path = BSpline<Vec2f>(lightPoints, 2, false, false);
            light.speed = lightSpeed;
        }
        if (darkActive) {
            dark.path = BSpline<Vec2f>(darkPoints, 2, false, false);
            dark.speed = darkSpeed;
        }
    }

    void update() {
        elapsed += 1.0f / FRAMERATE;

        if (lightActive && !light.active && elapsed > lightStart && !light.completed) {
            light.start();
        }
        if (darkActive && !dark.active && elapsed > darkStart && !dark.completed) {
            dark.start();
        }

        light.update();
        dark.update();

        //  ended?
        //if (elapsed > lightStart && elapsed > darkStart && !light.active && !dark.active) {
        if (light.completed && dark.completed) {
            stop();
        }
    }

    void stop() {
        light.stop();
        dark.stop();
        active = false;
        elapsed = 0;
    }
};

class PlanetController
{
public:
    Planet* lightPlanet;
    Planet* darkPlanet;

    vector<Formation*> formations;
    int currentFormation;
    bool active;

    PlanetController() {
        lightPlanet = new Planet(0.22f);
        darkPlanet = new Planet(0.22f);

        currentFormation = 0;
        setupFormations();
        start();
    }

    ~PlanetController() {
        delete lightPlanet;
        delete darkPlanet;
    }

    Formation& activeFormation() {
        assert(currentFormation < formations.size());
        return *(formations[currentFormation]);
    }

    void start() {
        //  start the 1st formation
        Formation& ff = activeFormation();
        ff.start();
    }

    void setupFormations() {
        Formation* ff;

        //  easy
        ff = new Formation(*lightPlanet, *darkPlanet);
        ff->lightPoints.push_back(Vec2f(5.0f, 0.15f));
        ff->lightPoints.push_back(Vec2f(0, 0.55f));
        ff->lightPoints.push_back(Vec2f(-5.0f, 0.15f));

        ff->darkPoints.push_back(Vec2f(5.0f, -0.2f));
        ff->darkPoints.push_back(Vec2f(0, -0.6f));
        ff->darkPoints.push_back(Vec2f(-5.0f, -0.2f));

        ff->lightActive = true;
        ff->lightSpeed = 0.1f;
        ff->lightStart = 0.05f;

        ff->darkActive = true;
        ff->darkSpeed = 0.1f;
        ff->darkStart = 0.5f;
        formations.push_back(ff);

        //  dipping formation
        ff = new Formation(*lightPlanet, *darkPlanet);
        ff->lightPoints.push_back(Vec2f(3.0f, 3.05f));
        ff->lightPoints.push_back(Vec2f(0, 0));
        ff->lightPoints.push_back(Vec2f(-3.0f, -3.05f));

        ff->darkPoints.push_back(Vec2f(3.0f, -3.05f));
        ff->darkPoints.push_back(Vec2f(0, 0));
        ff->darkPoints.push_back(Vec2f(-3.0f, 3.05f));

        ff->lightActive = true;
        ff->lightSpeed = 0.1f;
        ff->lightStart = 0.05f;

        ff->darkActive = true;
        ff->darkSpeed = 0.1f;
        ff->darkStart = 0;
        formations.push_back(ff);
        

        //  hugging formation
        ff = new Formation(*lightPlanet, *darkPlanet);
        // straight line
        for (int i=0; i < 10; ++i) {
            float yoff = 0.3 * sin(i/10 * M_PI);
            ff->lightPoints.push_back(Vec2f(3.0f - 6.0f*(i/10.0f), 0));
            ff->darkPoints.push_back(Vec2f(3.0f - 6.0f*(i/10.0f), 0.3f * sin(i/10.0f * 6 * M_PI)));
        }

        ff->lightActive = true;
        ff->lightSpeed = 0.1f;
        ff->lightStart = 0;

        ff->darkActive = true;
        ff->darkSpeed = 0.105f;
        ff->darkStart = 0.22f;
        formations.push_back(ff);


        //  kinda random
        ff = new Formation(*lightPlanet, *darkPlanet);
        ff->lightPoints.push_back(Vec2f(3.0f, 3.05f));
        ff->lightPoints.push_back(Vec2f(0.25f, -0.25f));
        ff->lightPoints.push_back(Vec2f(-0.25f, 0.25f));
        ff->lightPoints.push_back(Vec2f(-3.0f, -3.05f));

        ff->darkPoints.push_back(Vec2f(3.0f, -3.05f));
        ff->darkPoints.push_back(Vec2f(0.75f, 0.8f));
        ff->darkPoints.push_back(Vec2f(0, 0));
        ff->darkPoints.push_back(Vec2f(-0.65f, -0.8f));
        ff->darkPoints.push_back(Vec2f(-3.0f, 3.05f));

        ff->lightActive = true;
        ff->lightSpeed = 0.1f;
        ff->lightStart = 0.05f;

        ff->darkActive = true;
        ff->darkSpeed = 0.1f;
        ff->darkStart = 0;
        formations.push_back(ff);

        //  truly random
        for (int i=0; i < 100; ++i) {
            float lightLevel = Rand::randFloat(0, 1.0f);
            float darkLevel  = Rand::randFloat(0, 1.0f);
            float dir = Rand::randInt(0, 2) == 0 ? 1.0f : -1.0f;
            float dir2 = Rand::randInt(0, 2) == 0 ? 1.0f : -1.0f;
           
            ff = new Formation(*lightPlanet, *darkPlanet);
            ff->lightPoints.push_back(Vec2f(3.0f, dir* 3.05f));
            ff->lightPoints.push_back(Vec2f(0.25f, -dir * lightLevel));
            ff->lightPoints.push_back(Vec2f(-0.25f, dir * lightLevel));
            ff->lightPoints.push_back(Vec2f(-3.0f, dir * -3.05f));

            ff->darkPoints.push_back(Vec2f(3.0f, dir2 * 3.05f));
            ff->darkPoints.push_back(Vec2f(0.7f, dir2 * darkLevel));
            ff->darkPoints.push_back(Vec2f(-.7f, -dir2 * darkLevel));
            ff->darkPoints.push_back(Vec2f(-3.0f, dir2 * 3.05f));

            ff->lightActive = true;
            ff->lightSpeed = 0.1f;
            ff->lightStart = 0.05f;
            ff->lightStart = Rand::randFloat(0, 0.5f);

            ff->darkActive = true;
            ff->darkSpeed = 0.1f;
            ff->darkStart = Rand::randFloat(0, 0.5f);
            formations.push_back(ff);
        }
    }



    void update() {
        lightPlanet->collided = false;
        darkPlanet->collided = false;

        Formation& ff = activeFormation();
        ff.update();

        // XXX HACK, planet position update seems to get reset somewhere
        // if (!lightPlanet->active) {
        //     lightPlanet->position = Vec2f(100.0f, 100.0f);
        // }
        // if (!darkPlanet->active) {
        //     darkPlanet->position = Vec2f(100.0f, 100.0f);
        // }

        // XXX check for completion

        //  check formation completed?
        if (!ff.active) {
            //  Select the next formation
            if (GSPEED > 3.2f) {
                currentFormation = (currentFormation + 1) % formations.size();
            }
            else {
                currentFormation = 0;
            }

            activeFormation().start();
        }
    }

    void checkCollisions(Player& player) {
        if (player.position.distance(lightPlanet->position) < (lightPlanet->radius + player.radius)) {
            lightPlanet->collided = true;
        }
        else if (player.position.distance(darkPlanet->position) < (darkPlanet->radius + player.radius)) {
            darkPlanet->collided = true;
        }
    }
};

class TestApp : public AppBasic {
public:
    void prepareSettings( Settings *settings );
    void setup();
    void update();
    void draw();
    void keyDown( KeyEvent event );
	void cleanupExit();

    Config       mConfig;
    gl::Texture  mImage;
    CameraPersp  mCamera;
    gl::VboMesh  mVboMesh;
    gl::GlslProg mShader;
	PaStream*    mStream;
	SAMPLE*		 mAudioInput;
	Player		 mPlayer;
    PlanetController mPlanets;

    // Capture      mCapture;
private:
	void setupPortAudio();
};

static int audioCallback(const void *inputBuffer, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo* timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData )
{
    SAMPLE *out = (SAMPLE*)outputBuffer;
    const SAMPLE *in = (const SAMPLE*)inputBuffer;
    unsigned int i;
    (void) timeInfo; /* Prevent unused variable warnings. */
    (void) statusFlags;
    (void) userData;

    if( inputBuffer == NULL )
    {
        //for( i=0; i<framesPerBuffer; i++ )
        //{
        //    *out++ = 0;  /* left - silent */
        //    *out++ = 0;  /* right - silent */
        //}
        //gNumNoInputs += 1;
		audioScale = 0;
    }
    else
    {
        // Envelope follower from http://www.musicdsp.org/showone.php?id=97
        for( i=0; i<framesPerBuffer; i++ )
        {
            float input = *in++;
            // in++; // skip right input

            float envIn = abs(input);
            if (envelope < envIn)
            {
               envelope *= env_ga;
               envelope += (1-env_ga)*envIn;
            }
            else
            {
               envelope *= env_gr;
               envelope += (1-env_gr)*envIn;
            }
        }
        audioScale = envelope;
    }
    
    return paContinue;
}


void TestApp::prepareSettings( Settings *settings )
{
    settings->setWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);
	// settings->setFullScreenSize(SCREEN_WIDTH, SCREEN_HEIGHT);
	// settings->setFullScreen(true);
    settings->setFrameRate(FRAMERATE);
}

void TestApp::setupPortAudio()
{
	PaError err = Pa_Initialize();
    if( err != paNoError ) goto error;

    mConfig.init();

	/* -- setup input and output -- */
	PaStreamParameters inputParameters, outputParameters;

    // inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
    inputParameters.device = mConfig.getInputDeviceId(); /* default input device */
    inputParameters.channelCount = NUM_CHANNELS;
    inputParameters.sampleFormat = PA_SAMPLE_TYPE;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo( inputParameters.device )->defaultHighInputLatency ;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
    outputParameters.channelCount = NUM_CHANNELS;
    outputParameters.sampleFormat = PA_SAMPLE_TYPE;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultHighOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

	// alloc sample buffer
	//int numBytes = SAMPLE_FRAMES * sizeof(SAMPLE);
 //   mAudioInput = (SAMPLE *) malloc( numBytes );
	// mAudioInput = new SAMPLE[SAMPLE_FRAMES];

    /* -- setup stream -- */
    err = Pa_OpenStream(
              &mStream,
              &inputParameters,
              NULL, // XXX NO OUTPUT YET &outputParameters,
              PA_SAMPLE_RATE,
              FRAMES_PER_BUFFER,
              paClipOff,      /* we won't output out of range samples so don't bother clipping them */
              audioCallback,  /* no callback, use blocking API */
              NULL ); /* no callback, so no callback userData */
	if( err != paNoError ) goto error;

	// start audio subsystem
	err = Pa_StartStream( mStream );
	if( err != paNoError ) goto error;

error:
	if( err != paNoError ) {
		// XXX log it
        const char* errorText = Pa_GetErrorText(err);
        console() << "PortAudio error: " << errorText << std::endl;
	}
	else {
        console() << "PortAudio init OK!" << std::endl;
	}
}

void TestApp::setup()
{
	//  Check GL uniform limits
	int maxFragUniforms;
	// glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_BLOCKS, &maxFragUniforms);
    //gl::enableDepthRead();
    //gl::enableDepthWrite();

    gl::disableDepthRead();

    try {
        mShader = gl::GlslProg( loadResource( RES_VERT ), loadResource( RES_FRAG ) );
    }
    catch( gl::GlslProgCompileExc &exc ) {
        console() << "Shader compile error: " << std::endl;
        console() << exc.what();
    }
    catch( ... ) {
        console() << "Unable to load shader" << std::endl;
    }

	setupPortAudio();

    int cVertexes = 4;
    int cQuads = 1;

    gl::VboMesh::Layout layout;
    layout.setStaticIndices();
    layout.setStaticPositions();
    layout.setStaticTexCoords2d();

    mVboMesh = gl::VboMesh( cVertexes, cQuads * 4, layout, GL_QUADS );

    vector<uint32_t> indices;
    vector<Vec3f>    positions;
    vector<Vec2f>    texcoords;

	texcoords.push_back(Vec2f(0, 0));
	texcoords.push_back(Vec2f(1.0f, 0));
	texcoords.push_back(Vec2f(1.0f, 1.0f));
	texcoords.push_back(Vec2f(0, 1.0f));

	positions.push_back(Vec3f(0, 0, 0));
	positions.push_back(Vec3f(SCREEN_WIDTH, 0, 0));
	positions.push_back(Vec3f(SCREEN_WIDTH, SCREEN_HEIGHT, 0));
	positions.push_back(Vec3f(0, SCREEN_HEIGHT, 0));

	for (int i=0; i<4; ++i) {
		indices.push_back(i);
	}

    mVboMesh.bufferIndices( indices );
    mVboMesh.bufferPositions( positions );
    mVboMesh.bufferTexCoords2d( 0, texcoords );

    try {
        gl::Texture::Format format;
        format.setInternalFormat(GL_RGBA_FLOAT32_ATI);
        //  not required with newer cards
        format.setMagFilter(GL_NEAREST);
        format.setMinFilter(GL_NEAREST);
        mImage = gl::Texture( loadImage( loadResource( RES_TEXTURE_PNG ) ), format );
    }
    catch( ... ) {
        std::cout << "unable to load the texture file!" << std::endl;
    }

    // mCapture = getAvailableCapture(VWIDTH, VHEIGHT);
	// mCamera.lookAt( Vec3f( 0, 0, -100 ), Vec3f::zero() );

	CTimer.start();
}

void TestApp::update()
{
	mPlayer.update();
    mPlanets.update();
    mPlanets.checkCollisions(mPlayer);

    // XXX adjust only on transitions
    if (mPlanets.lightPlanet->collided) {
        mPlayer.radius += 0.0015f;
    }
    else if (mPlanets.darkPlanet->collided) {
        mPlayer.radius -= 0.01f;
    }
    GSPEED = 1.0f + 50.0f * mPlayer.radius;
    mPlayer.radius = math<float>::clamp(mPlayer.radius, 0.02f, 0.15f);
}

//  Set an ortho view of -1,-1 -> 1,1
static void setup2dMatrices() {
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
    glOrtho( -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f );
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
    glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
}

void TestApp::draw()
{	
    gl::clear( Color( 0.15f, 0.15f, 0.15f ) );
    gl::setMatricesWindow(Vec2i(SCREEN_WIDTH, SCREEN_HEIGHT));

	// total elapsed time in seconds
	float elapsed = static_cast<float>(CTimer.getSeconds());

    // gl::setMatrices(mCamera);

    Vec2f playerOffset(0, 0.05*sin(elapsed));

    float playerPower = mPlayer.radius;
    playerPower += playerPower * 0.25f * sin(elapsed);

    float dist = mPlanets.darkPlanet->position.distance(mPlayer.position);
    float att = math<float>::clamp(4.0f * dist * dist, 0.0f, 1.0f);
    playerPower *= att;

    //  Shader draws background
    mImage.bind(0);
	//mImage.enableAndBind();
    mShader.bind();
    mShader.uniform("resolution", Vec2f(SCREEN_WIDTH, SCREEN_HEIGHT));
    // mShader.uniform("playerPos", Vec2f(0, 0) + Vec2f(0, 0.08*sin(elapsed)));
    mShader.uniform("playerInput", playerPower);
	mShader.uniform("playerPos", mPlayer.position+playerOffset);
	mShader.uniform("darkInput", 0.044f);
    mShader.uniform("darkPos", mPlanets.darkPlanet->position);
	mShader.uniform("lightInput", 0.11f);
    mShader.uniform("lightPos", mPlanets.lightPlanet->position);
    mShader.uniform("tex", 0);
    mShader.uniform("time", static_cast<float>(CTimer.getSeconds()));

    gl::draw( mVboMesh );

    mShader.unbind();
    mImage.unbind();

    // //  Check collisions
    // ColorA playerColor(0.3f, 0.3f, 0.3f, 1.0f);
    // if (mPlanets.darkPlanet->collided) {
    //     playerColor = ColorA(1.0f, 0, 0, 1.0f);
    // }
    // else if (mPlanets.lightPlanet->collided) {
    //     playerColor = ColorA(0, 1.0f, 0, 1.0f);
    // }

    // //  Draw planets
    // setup2dMatrices();
    // gl::color(playerColor);
    // gl::drawSolidCircle(mPlayer.position+playerOffset, 0.015+0.1f*envelope, 16);
    // // gl::drawSolidCircle(Vec2f(100.0f, 100.0f), 100.0f, 32);
}

void TestApp::keyDown( KeyEvent event )
{
    if (event.getCode() == app::KeyEvent::KEY_ESCAPE) {
		cleanupExit();
    }
}

void TestApp::cleanupExit()
{
	PaError err = Pa_StopStream( mStream );
    if( err != paNoError )
		printf(  "PortAudio error: %s\n", Pa_GetErrorText( err ) );
	err = Pa_Terminate();
	if( err != paNoError )
		printf(  "PortAudio error: %s\n", Pa_GetErrorText( err ) );
    this->quit();
}

CINDER_APP_BASIC( TestApp, RendererGl )

