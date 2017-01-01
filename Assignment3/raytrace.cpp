//
// template-rt.cpp
//

#define _CRT_SECURE_NO_WARNINGS
#include "matm.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

using namespace std;

// global constants
#define MAX_SPHERES 5
#define MAX_LIGHTS 5
#define MIN_HIT_TIME 1.0f
#define MIN_RELECT_HIT_TIME 0.0001f
#define MAX_REFLECTIONS 3

// Resolution
int g_width;
int g_height;

// Structures
struct Ray
{
    vec4 origin;
    vec4 dir;
    int reflectionLevel;
};

struct Sphere {
    string id;
    vec4 position;
    vec3 scale;
    vec4 color;
    float a;
    float d;
    float s;
    float r;
    float specularExponent;
    mat4 inverseTransform;
};

struct Light {
    string id;
    vec4 position;
    vec4 color;
};

struct Intersection {
    Ray ray;
    float d;
    vec4 point;
    bool intPoint;
    Sphere *sphere;
    vec4 normal;
};

vector<vec4> g_colors;

// Planes data
float g_left;
float g_right;
float g_top;
float g_bottom;
float g_near;


vector<Sphere> g_spheres;
vector<Light> g_lights;
vec4 g_backgroundColor;
vec4 g_ambientIntensity;
string g_outputFilename;


// -------------------------------------------------------------------
// Input file parsing

vec4 toVec4(const string& s1, const string& s2, const string& s3)
{
    stringstream ss(s1 + " " + s2 + " " + s3);
    vec4 result;
    ss >> result.x >> result.y >> result.z;
    result.w = 1.0f;
    return result;
}

float toFloat(const string& s)
{
    stringstream ss(s);
    float f;
    ss >> f;
    return f;
}

void parseLine(const vector<string>& vs)
{
    if (vs[0].empty()) {
        return;
    }
    
    if (vs[0] == "NEAR") {
        g_near = toFloat(vs[1]);
    }
    else if (vs[0] == "LEFT"){
        g_left = toFloat(vs[1]);
    }
    else if (vs[0] == "RIGHT"){
        g_right = toFloat(vs[1]);
    }
    else if (vs[0] == "TOP"){
        g_top = toFloat(vs[1]);
    }
    else if (vs[0] == "BOTTOM"){
        g_bottom = toFloat(vs[1]);
    }
    else if (vs[0] == "RES"){
        g_width = (int) toFloat(vs[1]);
        g_height = (int) toFloat(vs[2]);
        g_colors.resize((unsigned int) (g_width * g_height));
    }
    else if (vs[0] == "SPHERE"){
        if (g_spheres.size() < MAX_SPHERES) {
                            Sphere sphere;
                            sphere.id = vs[1];
                            sphere.position = toVec4(vs[2], vs[3], vs[4]);
                            sphere.scale = vec3(toFloat(vs[5]), toFloat(vs[6]), toFloat(vs[7]));
                            sphere.color = toVec4(vs[8], vs[9], vs[10]);
                            sphere.a = toFloat(vs[11]);
                            sphere.d = toFloat(vs[12]);
                            sphere.s = toFloat(vs[13]);
                            sphere.r = toFloat(vs[14]);
                            sphere.specularExponent = toFloat(vs[15]);
                            InvertMatrix(Scale(sphere.scale), sphere.inverseTransform);
                            
                            g_spheres.push_back(sphere);
                        }

    }
    else if (vs[0] == "LIGHT"){
        if (g_lights.size() < MAX_LIGHTS) {
                            Light light;
                            light.id = vs[1];
                            light.position = toVec4(vs[2], vs[3], vs[4]);
                            light.color = toVec4(vs[5], vs[6], vs[7]);
            
                            g_lights.push_back(light);
                        }
    }
    else if (vs[0] == "BACK"){
        g_backgroundColor = toVec4(vs[1], vs[2], vs[3]);
    }
    else if (vs[0] == "AMBIENT"){
        g_ambientIntensity = toVec4(vs[1], vs[2], vs[3]);
    }
    else if (vs[0] == "OUTPUT"){
        g_outputFilename = vs[1];
    }
}

void loadFile(const char* filename)
{
    ifstream is(filename);
    if (is.fail())
    {
        cout << "Could not open file " << filename << endl;
        exit(1);
    }
    string s;
    vector<string> vs;
    while(!is.eof())
    {
        vs.clear();
        getline(is, s);
        istringstream iss(s);
        while (!iss.eof())
        {
            string sub;
            iss >> sub;
            vs.push_back(sub);
        }
        parseLine(vs);
    }
}


// -------------------------------------------------------------------
// Utilities

void setColor(int ix, int iy, const vec4& color)
{
    int iy2 = g_height - iy - 1; // Invert iy coordinate.
    g_colors[iy2 * g_width + ix] = color;
}


// -------------------------------------------------------------------
// Intersection routine

//Find the nearest sphere intersection of a ray.
Intersection calculateNearestIntersection(const Ray &ray) {
    Intersection intrsct;
    intrsct.ray = ray;
    intrsct.d = -1;
    intrsct.intPoint = false;
    
    for (Sphere &sphere : g_spheres) {
        vec4 S = sphere.inverseTransform * (sphere.position - ray.origin);
        vec4 C = sphere.inverseTransform * ray.dir;
        
        // According to the quadratic equation:
        // |c|^2t^2 + 2(S.tc) + |S|^2 - 1
        float a = dot(C, C);
        float b = dot(S, C);
        float c = dot(S, S) - 1;

        float solution;
        float discriminant = b * b - a * c;
        
        bool intPoint = false;
        
        if (discriminant < 0) {
            // Do not intersect
            continue;
        } else if (discriminant == 0) {
            // Intersect at one point
            solution = b / a;
        } else {
            // Line intersecvt at two points
            float root = sqrtf(discriminant);
            float solution1 = (b - root) / a;
            float solution2 = (b + root) / a;
            
            // Use the minimum
            solution = fminf(solution1, solution2);
            if (solution <= MIN_RELECT_HIT_TIME || (ray.reflectionLevel == 0 && solution <= MIN_HIT_TIME)) {
                solution = fmaxf(solution1, solution2);
                intPoint = true;
            }
        }
        
        // Validate the Solution
        if (solution <= MIN_RELECT_HIT_TIME || (ray.reflectionLevel == 0 && solution <= MIN_HIT_TIME)) {
            continue;
        }
        
        if ((intrsct.d == -1 || solution < intrsct.d)) {
            intrsct.d = solution;
            intrsct.sphere = &sphere;
            intrsct.intPoint = intPoint;
        }
    }
    
    // Find the intersection and normal
    if (intrsct.d != -1) {
        intrsct.point = ray.origin + ray.dir * intrsct.d;
        
        vec4 normal = intrsct.point - intrsct.sphere->position;
        // Invert normal for intPoint
        if (intrsct.intPoint) {
            normal = -normal;
        }
        
        mat4 trans = transpose(intrsct.sphere->inverseTransform);
        normal = trans * intrsct.sphere->inverseTransform * normal;
        normal.w = 0;
        intrsct.normal = normalize(normal);
    }
    
    return intrsct;
}


// -------------------------------------------------------------------
// Ray tracing

vec4 trace(const Ray& ray)
{
    // Limit reflection level
    if (ray.reflectionLevel >= MAX_REFLECTIONS) {
        return vec4();
    }
    
    // Find the nearest intersecting sphere
    Intersection intrsct = calculateNearestIntersection(ray);
    
    if (intrsct.d == -1 && ray.reflectionLevel == 0) // no intersection and is an initial ray
    {
        return g_backgroundColor;
    } else if (intrsct.d == -1) // no intersection and not an initial ray
    {
        return vec4();
    }
    
    // Initial intersection color (with ambience)
    vec4 color = intrsct.sphere->color * intrsct.sphere->a * g_ambientIntensity;
    
    // Phong shading
    vec4 diffusion = vec4(0, 0, 0, 0);
    vec4 specular = vec4(0, 0, 0, 0);
    
    for (Light light : g_lights) // Generate ray from intersection to light
    {
        Ray lightRay;
        lightRay.origin = intrsct.point;
        lightRay.dir = normalize(light.position - intrsct.point);
        
        // Determine if the light source is not obstructed
        Intersection lightintrsct = calculateNearestIntersection(lightRay);
        if (lightintrsct.d == -1) {
            // Diffuse Light
            float diffusionIntensity = dot(intrsct.normal, lightRay.dir);
            if (diffusionIntensity > 0) {
                diffusion += diffusionIntensity * light.color * intrsct.sphere->color;
                
                // Half vector between lightRay and ray
                vec4 H = normalize(lightRay.dir - ray.dir);
                
                // Specular Light
                float specularIntensity = dot(intrsct.normal, H);
                specular += powf(powf(specularIntensity, intrsct.sphere->specularExponent), 3) * light.color;
            }
        }
    }
    
    // Apple the values calculated to color
    color += diffusion * intrsct.sphere->d + specular * intrsct.sphere->s;
    
    // Get Reflection
    Ray reflectRay;
    reflectRay.origin = intrsct.point;
    reflectRay.dir = normalize(ray.dir - 2.0f * intrsct.normal * dot(intrsct.normal, ray.dir));
    reflectRay.reflectionLevel = ray.reflectionLevel + 1;
    color += trace(reflectRay) * intrsct.sphere->r;
    
    return color;

}

vec4 getDir(int ix, int iy)
{
    
    return vec4(g_left + ((float) ix / g_width) * (g_right - g_left), g_bottom + ((float) iy / g_height) * (g_top - g_bottom), -g_near, 0.0f);
}

void renderPixel(int ix, int iy)
{
    Ray ray;
    ray.origin = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    ray.dir = getDir(ix, iy);
    ray.reflectionLevel = 0;
    vec4 color = trace(ray);
    setColor(ix, iy, color);
}

void render()
{
    for (int iy = 0; iy < g_height; iy++)
        for (int ix = 0; ix < g_width; ix++)
            renderPixel(ix, iy);
}


// -------------------------------------------------------------------
// PPM saving

void savePPM(int Width, int Height, const char* fname, unsigned char* pixels)
{
    FILE *fp;
    const int maxVal=255;
    
    printf("Saving image %s: %d x %d\n", fname, Width, Height);
    fp = fopen(fname,"wb");
    if (!fp) {
        printf("Unable to open file '%s'\n", fname);
        return;
    }
    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", Width, Height);
    fprintf(fp, "%d\n", maxVal);
    
    for(int j = 0; j < Height; j++) {
        fwrite(&pixels[j*Width*3], 3, Width, fp);
    }
    
    fclose(fp);
}

void saveFile()
{
    // Convert color components from floats to unsigned chars.
    unsigned char* buf = new unsigned char[g_width * g_height * 3];
    for (int y = 0; y < g_height; y++)
        for (int x = 0; x < g_width; x++)
            for (int i = 0; i < 3; i++) {
                float color = ((float *) g_colors[y * g_width + x])[i];
                color = fminf(color, 1); // Clamp color value to 1
                buf[y * g_width * 3 + x * 3 + i] = (unsigned char) (color * 255.9f);
            }
    savePPM(g_width, g_height, g_outputFilename.c_str(), buf);
    delete[] buf;
}


// -------------------------------------------------------------------
// Main

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        cout << "Usage: template-rt <input_file.txt>" << endl;
        exit(1);
    }
    loadFile(argv[1]);
    render();
    saveFile();
    return 0;
}
