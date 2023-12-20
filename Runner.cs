using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using GraphicsEngine;
using System.Diagnostics;
using PhysicsEngine.Physics;
using System.Collections.Generic;

namespace PhysicsEngine
{
    public class Runner : Game
    {
        private GraphicsDeviceManager _graphics;
        private ShapeDrawer shapeDrawer;
        private World world;

        public Runner()
        {   
            _graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";
            IsMouseVisible = true;
            
        }

        protected override void Initialize()
        {
            base.Initialize();
        }

        protected override void LoadContent()
        {
            this.shapeDrawer = new ShapeDrawer(this);
            this.world = new World();

            /*            Random rnd = new Random();
                        for (int i = 0; i < 10; i++)
                        {
                            int randX = rnd.Next(1, 79) * 10;
                            int randY = rnd.Next(1, 47) * 10;
                            world.AddBox(randX, randY, 50, 50);
                        }*/
            world.AddBox(100, 100, 100, 100);
            PolygonalRigidBody box2 = world.AddBox(300, 100, 100, 100);
            box2.Rotation = MathHelper.PiOver4;


        }

        protected override void Update(GameTime gameTime)
        {


            // TODO: Add your update logic here
            PolygonalRigidBody player = world.Bodies[0];
            KeyboardState ks = Keyboard.GetState();
            
            Vector2 v = Vector2.Zero;
            if (ks.IsKeyDown(Keys.A)) {

                v += new Vector2(-1, 0);
            }
            if (ks.IsKeyDown(Keys.D)) {
                v += new Vector2(1, 0);
            }
            if (ks.IsKeyDown(Keys.W)) {
                v += new Vector2(0, 1);
            }
            if (ks.IsKeyDown(Keys.S)) {
                v += new Vector2(0, -1);
            }

            v *= 50;
            player.Velocity = v;
            

            player.AngularVelocity = MathHelper.TwoPi / 10;
            this.world.Step((float) gameTime.ElapsedGameTime.TotalSeconds);

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            shapeDrawer.Begin();
            foreach (PolygonalRigidBody body in world.Bodies)
            {
                Color outline = Color.Black;
                if (body.colliding)
                {
                    outline = Color.Red;
                }
                shapeDrawer.DrawConvexPolygon(body.GetGlobalPoints(), Color.Red, outline, Common.LINETHICKNESS);
                shapeDrawer.DrawFilledCircle(body.Position, 3, 5, Color.White);

            }
            shapeDrawer.End();


            base.Draw(gameTime);
        }
    }
}
