#include <stdio.h>
#include "linear.h"

using namespace r3;

static void PrintPoint( const char * label, const Vec4f & p ) {
  printf( "%s = [ %8.3f %8.3f %8.3f %8.3f ]\n", label, p.x, p.y, p.z, p.w );
}

int main(int /*argc*/, char** /*argv*/) {
  Posef worldFromQuad;
  Posef worldFromCam;

  worldFromQuad.r = Quaternionf(Vec3f(0, 1, 0), ToRadians(30.0f));
  worldFromQuad.t = Vec3f( 0.5, 0.5, 1.0f);

  worldFromCam.t = Vec3f(0, 0.5, -1.0f );

  Matrix4f worldFromQuadM = worldFromQuad.GetMatrix4();
  Matrix4f camFromWorldM = worldFromCam.Inverted().GetMatrix4();

  Matrix4f camFromQuadM = camFromWorldM * worldFromQuadM;
  Matrix4f quadFromCamM = camFromQuadM.Inverted();

  constexpr int width = 80;
  constexpr int height = 40;
  Matrix4f clipFromCamM = Perspective( 60.0f, float(width)/height, 0.1f, 10.0f );
  Matrix4f camFromClipM = clipFromCamM.Inverted();
  for (int j = 0; j < height; j++ ) {
    for (int i = 0; i < width; i++) {
      Vec4f clipPoint( 2 * i / float(width-1) - 1.0f, 2 * j / float(height-1) - 1.0f, 0, 1);
      Vec4f camPoint = camFromClipM * clipPoint;
      PrintPoint( "camPoint h", camPoint );
      camPoint /= camPoint.w;
      PrintPoint( "camPoint r", camPoint );
      Vec4f quadPoint = quadFromCamM * camPoint;
      PrintPoint( "quadPoint r", quadPoint );

    }
  }


  return 0;
}
