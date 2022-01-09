#include "Image.h"
#include "gtest/gtest.h"
#include "Types.h"

namespace{
  
TEST(Image,all){

  //test defaultConstruct
  sf::Image img1;
  EXPECT_EQ(img1.rows,0);
  EXPECT_EQ(img1.cols,0);
  //EXPECT_TRUE(img1.empty());

  sf::Image img2(100,50);
  EXPECT_EQ(img2.rows,100);
  EXPECT_EQ(img2.cols,50);
  EXPECT_EQ(img2(0,0),0);
  EXPECT_EQ(img2(99,49),0);

  uint32_t x=33, y=44;
  EXPECT_EQ(img2(x, y), 0);

  sf::Image img3(200,100,33);
  EXPECT_EQ(img3.rows,200);
  EXPECT_EQ(img3.cols,100);
  EXPECT_EQ(img3(0,0),33);
  EXPECT_EQ(img3(99,49),33);
  EXPECT_EQ(img3(199,99),33);

  sf::Mat<uint8_t,8,8> mat1=sf::Mat<uint8_t,8,8>::Identity();
  sf::Image img4(mat1);
  EXPECT_EQ(img4.rows,8);
  EXPECT_EQ(img4.cols,8);
  EXPECT_EQ(img4(0,0),1);
  EXPECT_EQ(img4(7,7),1);
  EXPECT_EQ(img4(1,0),0);

  sf::Image img5(img4);
  EXPECT_EQ(img5.rows,8);
  EXPECT_EQ(img5.cols,8);
  EXPECT_EQ(img5(0,0),1);
  EXPECT_EQ(img5(7,7),1);
  EXPECT_EQ(img5(1,0),0);

  sf::Image img6=img4;
  EXPECT_EQ(img6.rows,8);
  EXPECT_EQ(img6.cols,8);
  EXPECT_EQ(img6(0,0),1);
  EXPECT_EQ(img6(7,7),1);
  EXPECT_EQ(img6(1,0),0);
}        

}  //namespace
