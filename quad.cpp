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

int main(int /*argc*/, char** /*argv*/) {
  Posef worldFromQuad;
  Posef worldFromCam;

  // turn the quad around so its -z is pointing toward the camera
  worldFromQuad.r = Quaternionf(Vec3f(0, 1, 0), ToRadians(0.0f));
  worldFromQuad.t = Vec3f(0, 0, -4.0f);

  worldFromCam.t = Vec3f(0, 0, 1.0f);

  Matrix4f worldFromQuadM = worldFromQuad.GetMatrix4();
  Matrix4f camFromWorldM = worldFromCam.Inverted().GetMatrix4();

  Matrix4f camFromQuadM = camFromWorldM * worldFromQuadM;
  Matrix4f quadFromCamM = camFromQuadM.Inverted();

  constexpr int width = 80;
  constexpr int height = 40;
  Matrix4f clipFromCamM = Perspective(60.0f, float(width) / height, 1.0f, 10.0f);
  Matrix4f clipFromQuadM = clipFromCamM * camFromQuadM;

  Matrix4f camFromClipM = clipFromCamM.Inverted();

  //PrintMatrix("I", clipFromCamM * camFromClipM);
  //printf("\n");
  //PrintMatrix("I", quadFromCamM * camFromClipM * clipFromCamM * camFromQuadM);

  Vec4f quadPlaneInCam = quadFromCamM.Transposed() * Vec4f(0, 0, 1, 0);

  PrintPoint("quadPlaneInCam", quadPlaneInCam);

  Matrix4f newClipFromCamM = clipFromCamM;
  for (int i = 0; i < 4; i++) {
    newClipFromCamM.el(2, i) = quadPlaneInCam.v[i] - newClipFromCamM.el(3, i);
  }

  Matrix4f camFromNewClipM = newClipFromCamM.Inverted();

  PrintPoint("quad(cam(newclip(cam(quad(zero)))))",
             quadFromCamM * camFromNewClipM * newClipFromCamM * camFromQuadM * Vec3f(0, 0, 0));

    for (int j = 0; j < height; j++) {
      for (int i = 0; i < width; i++) {
        Vec4f clipPoint(2 * i / float(width - 1) - 1.0f, 2 * j / float(height - 1) - 1.0f, -1, 1);
        //PrintPoint( "clipPoint", clipPoint);
        Vec4f camPoint = camFromNewClipM * clipPoint;
        //PrintPoint( "camPoint h", camPoint);
        Vec4f camPointR = camPoint;
        camPointR /= camPointR.w;
        //PrintPoint( "camPoint r", camPointR);
        Vec4f quadPoint = quadFromCamM * camPointR;
        //PrintPoint( "quadPoint r", quadPoint);
        bool inside = ((-quadPoint.w < quadPoint.x && quadPoint.x < quadPoint.w) && (-quadPoint.w < quadPoint.y && quadPoint.y < quadPoint.w));
        printf("%s", inside ? "." : " ");
      }
      printf("\n");
    }

  PrintPoint("quadPlaneInCam", quadPlaneInCam);

  return 0;
}
