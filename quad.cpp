#include <stdio.h>
#include <strings.h>
#include <string.h>
#include "linear.h"
#include "stb.h"

using namespace r3;

static void PrintPoint(const char* label, const Vec4f& p) {
  printf("%s = [ %8.3f %8.3f %8.3f %8.3f ]\n", label, p.x, p.y, p.z, p.w);
}

static void PrintPoint(const char* label, const Vec3f& p) {
  printf("%s = [ %8.3f %8.3f %8.3f ]\n", label, p.x, p.y, p.z);
}

static void PrintMatrix(const char* label, const Matrix4f& m) {
  const char *empty = "";
  char fmt[128];
  int ll = strlen(label);
  sprintf(fmt, "%%%ds %%d | %%8.3f %%8.3f %%8.3f %%8.3f |\n", ll);
  for (int i = 0; i < 4; i++) {
    printf(fmt, i == 0 ? label : empty, i, m.el(i, 0), m.el(i, 1), m.el(i, 2), m.el(i, 3));
  }
}

#define PP(p) PrintPoint(#p, p)
#define PM(m) PrintMatrix(#m, m)

Matrix4f computeClipFromQuad() {

  Posef worldFromQuad;
  Posef worldFromCam;

  // turn the quad around so its -z is pointing toward the camera
  worldFromQuad.r = Quaternionf(Vec3f(0.25f, 1, 0), ToRadians(60.0f));
  worldFromQuad.t = Vec3f(0.5f, 0.1f, -1.0f);

  worldFromCam.t = Vec3f(0, 0, 1.0f);

  Matrix4f worldFromQuadM = worldFromQuad.GetMatrix4();
  Matrix4f camFromWorldM = worldFromCam.Inverted().GetMatrix4();

  Matrix4f camFromQuadM = camFromWorldM * worldFromQuadM;
  Matrix4f quadFromCamM = camFromQuadM.Inverted();

  Matrix4f clipFromCamM = Perspective(90.0f, 1.0f, 1.0f, 10.0f);
  Matrix4f clipFromQuadM = clipFromCamM * camFromQuadM;

  return clipFromQuadM;
}

void AdjustForwardTransform( Matrix4f & clipFromQuadM) {
  // We force the transform from quad space to clip space to leave
  // z unaltered. This ensures the inverse transform transforms
  // back into that plane.
  clipFromQuadM.SetRow(2, Vec4f( 0, 0, 1, 0 ) );
}


int main(int /*argc*/, char** /*argv*/) {
  Matrix4f quadFromClipM = []() {
    Matrix4f clipFromQuadM = computeClipFromQuad();
    AdjustForwardTransform( clipFromQuadM );
    Matrix4f quadFromClipM = clipFromQuadM.Inverted();
    // We don't care about the z coordinate, so zero row and col 2
    Vec4f z;
    z *= 0;
    quadFromClipM.SetRow(2, z);
    quadFromClipM.SetColumn(2, z);
    return quadFromClipM;
  }();

  PM(quadFromClipM);

  constexpr int width = 640;
  constexpr int height = 480;
  constexpr float aspect = float(width)/height;

  unsigned char* img = new unsigned char[width * height * 3];
  bzero( img, width * height * 3);

  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width; i++) {
      Vec4f clipPoint( aspect * (2 * i / float(width - 1) - 1.0f), 2 * j / float(height - 1) - 1.0f, 1234.5f /*don't care*/, 1);
      Vec4f quadPoint = quadFromClipM * clipPoint;
      if (i == (width/2) && j == (height/2)) {
        PP(clipPoint);
        PP(quadPoint);
      }
      quadPoint /= quadPoint.w;

      bool inside = ((-quadPoint.w < quadPoint.x && quadPoint.x < quadPoint.w) &&
                     (-quadPoint.w < quadPoint.y && quadPoint.y < quadPoint.w));
      if (inside) {
        quadPoint *= 0.5f;
        quadPoint += Vec4f( 0.5f, 0.5f, 0.5f, 0.0f);
        quadPoint = Max( quadPoint, Vec4f(0, 0, 0, 0) );
        quadPoint = Min( quadPoint, Vec4f(1, 1, 1, 1) );
        img[ (j * width + i ) * 3 + 0 ] = 255.99 * quadPoint.x;
        img[ (j * width + i ) * 3 + 1 ] = 255.99 * quadPoint.y;
      } else {
        img[ (j * width + i ) * 3 + 2 ] = 255.99;        
      }
    }
  }

  image_store_png("out.png", width, height, 3, img );
  delete[] img;

  return 0;
}
