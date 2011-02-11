uniform vec2 resolution;

// *Input is the metaball power

uniform float playerInput;
uniform vec2  playerPos;

uniform float darkInput;
uniform vec2  darkPos;

uniform float lightInput;
uniform vec2  lightPos;

uniform float time;

uniform sampler2D tex;

vec4 background(vec2 screenPos)
{
    vec2 p = screenPos;
    vec2 uv;
   
    float a = atan(p.y,p.x);
    float r = sqrt(dot(p,p));

    uv.x =          7.0*a/3.1416;
    uv.y = -time+ sin(7.0*r+time) + .7*cos(time+7.0*a);

    float w = .5+.5*(sin(time+7.0*r)+ .7*cos(time+7.0*a));

    vec3 col =  texture2D(tex,uv*.5).xyz;

    return vec4(col*w,1.0);
}

void main(void)
{
	// XXX WTF?  why is gl_FragCoord yielding negative values (on laptop only)???
    //  screen coords [-1,1], [-1,1]
    // vec2 screenPos;
    // screenPos.x = (gl_FragCoord.x / resolution.x)*2.0-1.0;
    // screenPos.y = (gl_FragCoord.y / resolution.y)*2.0;

    vec2 screenPos = (gl_FragCoord.xy / resolution.xy)*2.0-vec2(1.0,1.0);
    
    float playerLight = playerInput / distance(screenPos, playerPos);
    float light = lightInput / distance(screenPos, lightPos);    
    float dark =  darkInput / distance(screenPos, darkPos);
    
    vec4 playerLightCol = vec4(1.0, 1.0, 1.0, 1.0) * playerLight;
    vec4 lightCol = vec4(1.0, 1.0, 1.0, 1.0) * light * smoothstep(0.075, 0.15, light);
    vec4 darkCol  = vec4(1.0, 1.0, 1.0, 1.0) * dark * smoothstep(0.07, 1.0, dark);
    
    // gl_FragColor = (0.4 + (1.0 - length(screenPos))) * texture2D(tex, gl_TexCoord[0].xy) + vec4(val);
    // gl_FragColor = texture2D(tex, gl_TexCoord[0].xy) + lightCol - darkCol;
    // gl_FragColor = vec4(0.6, 0.6, 0.6, 1.0) + playerLightCol + 0.5f * lightCol - darkCol;
    // gl_FragColor = texture2D(tex, gl_TexCoord[0].xy) + playerLightCol + 0.5f * lightCol - darkCol;
    gl_FragColor = 0.5*background(screenPos) + texture2D(tex, gl_TexCoord[0].xy) + playerLightCol + 0.5 * lightCol - darkCol;
    // gl_FragColor = background(screenPos) + playerLightCol + 0.5 * lightCol - darkCol;
    
}

