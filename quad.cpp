#include <stdio.h>
#include "linear.h"

using namespace r3;

static void PrintPoint(const char* label, const Vec4f& p) {
  printf("%s = [ %8.3f %8.3f %8.3f %8.3f ]\n", label, p.x, p.y, p.z, p.w);
}

static void PrintPoint(const char* label, const Vec3f& p) {
  printf("%s = [ %8.3f %8.3f %8.3f ]\n", label, p.x, p.y, p.z);
}

static void PrintMatrix(const char* label, const Matrix4f& m) {
  for (int i = 0; i < 4; i++) {
    printf("%s %d | %8.3f %8.3f %8.3f %8.3f |\n", label, i, m.el(i, 0), m.el(i, 1), m.el(i, 2), m.el(i, 3));
  }
}

Matrix4f computeClipFromQuad( int width, int height ) {

  Posef worldFromQuad;
  Posef worldFromCam;

  // turn the quad around so its -z is pointing toward the camera
  worldFromQuad.r = Quaternionf(Vec3f(0.0f, 1, 0), ToRadians(40.0f));
  worldFromQuad.t = Vec3f(0, 0, -4.0f);

  worldFromCam.t = Vec3f(0, 0, 1.0f);

  Matrix4f worldFromQuadM = worldFromQuad.GetMatrix4();
  Matrix4f camFromWorldM = worldFromCam.Inverted().GetMatrix4();

  Matrix4f camFromQuadM = camFromWorldM * worldFromQuadM;
  Matrix4f quadFromCamM = camFromQuadM.Inverted();

  Matrix4f clipFromCamM = Perspective(60.0f, (width * 0.25f) / height, 1.0f, 10.0f);
  Matrix4f clipFromQuadM = clipFromCamM * camFromQuadM;

  return clipFromQuadM;
}

void AdjustForQuadPlane( Matrix4f & clipFromQuadM, Matrix4f & quadFromClipM) {

  // Planes are transformed by the inverse transpose of the transforms that transform points.
  // This transforms the forward XY plane at the origin in quad space into
  // clip space.
  // That plane is then used to replace the z row of the matrix.
  Vec4f quadPlaneInClip = quadFromClipM.Transposed() * Vec4f(0, 0, 1, 0);

  for (int i = 0; i < 4; i++) {
    clipFromQuadM.el(2, i) = quadPlaneInClip.v[i] - clipFromQuadM.el(3, i);
  }

  quadFromClipM = clipFromQuadM.Inverted();  

}


int main(int /*argc*/, char** /*argv*/) {
  constexpr int width = 80;
  constexpr int height = 40;

  Matrix4f clipFromQuadM = computeClipFromQuad( width, height );
  Matrix4f quadFromClipM = clipFromQuadM.Inverted();

  AdjustForQuadPlane( clipFromQuadM, quadFromClipM );

  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width; i++) {
      Vec4f clipPoint(2 * i / float(width - 1) - 1.0f, 2 * j / float(height - 1) - 1.0f, -1, 1);
      Vec4f quadPoint = quadFromClipM * clipPoint;
      bool inside = ((-quadPoint.w < quadPoint.x && quadPoint.x < quadPoint.w) &&
                     (-quadPoint.w < quadPoint.y && quadPoint.y < quadPoint.w));
      printf("%s", inside ? "." : " ");
    }
    printf("\n");
  }

  return 0;
}
