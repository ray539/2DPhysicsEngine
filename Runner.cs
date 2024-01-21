using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using GraphicsEngine;
using System.Diagnostics;
using PhysicsEngine.Physics;
using System.Collections.Generic;
using PhysicsEngine.Physics.ForceGenerators;

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

        public PolygonalRigidBody player;
        public PlayerForceGenerator playerForceGen;
        protected override void LoadContent()
        {
            this.shapeDrawer = new ShapeDrawer(this);
            this.world = new World();

            // world.AddBox(400, 120, 100, 100, false);
            world.AddPolygonalRigidBody(new List<Vector2>()
            {
                new Vector2(400, 120),
                new Vector2(500, 120),
                new Vector2(450, 220)
            }, false);

            world.AddBox(200, 120, 100, 100, false);
            world.AddBox(200, 220, 100, 100, false);
            world.AddBox(200, 320, 100, 100, false);


            world.AddBox(100, 100, 700, 20, true);
            world.AddBox(100, 120, 50, 300, true);
            world.AddBox(750, 120, 50, 300, true);
            // world.Bodies[4].Rotation = (float) Math.PI / 6;
            player = world.Bodies[0];

            this.playerForceGen = new PlayerForceGenerator(player);
            this.world.AddForceGenerator(this.playerForceGen);
        }

        
        protected override void Update(GameTime gameTime)
        {

            KeyboardState ks = Keyboard.GetState();
            Vector2 v = Vector2.Zero;
            if (ks.IsKeyDown(Keys.Space))
            {
                Common.PAUSE = true;
            }

            if (Common.PAUSE)
            {
                Debug.WriteLine("here");
            }

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

            v *= 1000 * player.mass;
            this.playerForceGen.SetForceVector(v);

            this.world.Step((float)gameTime.ElapsedGameTime.TotalSeconds, 2);
            

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            shapeDrawer.Begin();
            foreach (PolygonalRigidBody body in world.Bodies)
            {
                Color outline = Color.Black;
                if (!body.inmovable)
                {
                    shapeDrawer.DrawConvexPolygon(body.GetGlobalPoints(), Color.Red, outline, Common.LINETHICKNESS);
                } else
                {
                    shapeDrawer.DrawConvexPolygon(body.GetGlobalPoints(), Color.PaleGreen, outline, Common.LINETHICKNESS);
                }
                
                shapeDrawer.DrawFilledCircle(body.Position, 3, 5, Color.White);
            }


            foreach (CollusionData collusion in world.collusions)
            {
                shapeDrawer.DrawFilledCircle(collusion.contactPoint, 3, 5, Color.Orange);
            }

            shapeDrawer.End();




            base.Draw(gameTime);
        }
    }
}
