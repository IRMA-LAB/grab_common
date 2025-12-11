/**
 * @file TensionDistribution.cpp
 * @author Anna Berger
 * @date 13 Nov 2025
 * @brief File containing definitions of functions declared in TensionDistribution.h.
 */

#include "TensionDistribution.h"


namespace grabcdpr {

bool updateCablesTensionDistribution(RobotVars& vars)
{
  MatrixXd<8,1> tau_v;
  Index_and_limits Idx_and_lim{};

  MatrixXd<8,1> tau_m;
  double taumean=0.5*( Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
  tau_m.Fill(std::vector<double>{taumean,taumean,taumean,taumean,taumean,taumean,taumean,taumean});
  MatrixXd<6,8>A =vars.geom_jacobian_l.Transpose();
  Vector6d b=vars.platform.ext_load-A*tau_m;
  MatrixXd<8,6> AT=A.Transpose();
  MatrixXd<6,6> AAT=A*AT;
 // MatrixXd<8,6> MoorePenrose=AT*Inverse(AAT); //try catch block for inverse
  MatrixXd<8,6> MoorePenrose;
  try {
    MoorePenrose=AT*Inverse(AAT);
  }  catch (const std::runtime_error& e) {
    std::cout << "error:" << e.what() << std::endl;
    vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
    return false;
  }


  tau_v=MoorePenrose*b;

  MatrixXd<1,8> row_tau_v=tau_v.Transpose();
  double normatauv=Norm(row_tau_v);

  if (normatauv> 0.5*sqrt(8)*(Idx_and_lim.CTL_for_TD(2)-Idx_and_lim.CTL_for_TD(1)))
    {
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
      return false;
    }

    MatrixXd<8,1>  tau =tau_v+tau_m;


    if ((tau.Min()>=Idx_and_lim.CTL_for_TD(1)) && (tau.Max() <=Idx_and_lim.CTL_for_TD(2)))
    {
      vars.tension_vector=arma::vec(tau.Data(),8,1,true);
     // vars.tension_vector= toArmaMat_generic(tau);
      return true;
    }
    else if ((tau.Min()<Idx_and_lim.CTL_for_TD(1)) && (tau.Max()>Idx_and_lim.CTL_for_TD(2)))
    {
      double maxdiff=tau.Max()-Idx_and_lim.CTL_for_TD(2);
      double mindiff=Idx_and_lim.CTL_for_TD(1)-tau.Min();

      if (maxdiff>mindiff)
      {
        Idx_and_lim.indices_to_set(1)=tau.MaxIdx();
        Idx_and_lim.limits_to_set(1)=Idx_and_lim.CTL_for_TD(2);
      } else
      {
        Idx_and_lim.indices_to_set(1)=tau.MinIdx();
        Idx_and_lim.limits_to_set(1)=Idx_and_lim.CTL_for_TD(1);
      }
    }
    else if (tau.Max()>Idx_and_lim.CTL_for_TD(2))
    {
      Idx_and_lim.indices_to_set(1)=tau.MaxIdx();
      Idx_and_lim.limits_to_set(1)=Idx_and_lim.CTL_for_TD(2);
    }
    else if (tau.Min()<Idx_and_lim.CTL_for_TD(1))
    {
      Idx_and_lim.indices_to_set(1)=tau.MinIdx();
      Idx_and_lim.limits_to_set(1)=Idx_and_lim.CTL_for_TD(1);
    }

// try fixing non acceptable value to limit
    //adjust
    MatrixXd<7,1> tau_v7;
    MatrixXd<7,1> tau_m7;
    tau_m7.Fill(std::vector<double>{taumean,taumean,taumean,taumean,taumean,taumean,taumean});
    MatrixXd<6,7> smallerA;

    uint counter=0;
    for (uint i=1;i<9;++i)
    {
      if (!(i==Idx_and_lim.indices_to_set(1)))
      {
       counter=counter+1;
       smallerA.SetCol(counter,A.GetCol(i));
      }
    }


    b=vars.platform.ext_load-smallerA*tau_m7-A.GetCol(Idx_and_lim.indices_to_set(1))*Idx_and_lim.limits_to_set(1);
    MatrixXd<7,6> smallerAT=smallerA.Transpose();
    MatrixXd<6,6> SA_SAT=smallerA*smallerAT;
   // MatrixXd<7,6> smallerMoorePenrose=smallerAT*Inverse(SA_SAT); //try catch block for inverse
    MatrixXd<7,6> smallerMoorePenrose;
    try {
      smallerMoorePenrose=smallerAT*Inverse(SA_SAT);
    }  catch (const std::runtime_error& e) {
      std::cout << "error:" << e.what() << std::endl;
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
      return false;
    }


    tau_v7=smallerMoorePenrose*b;

    MatrixXd<1,7> row_tau_v7=tau_v7.Transpose();
    double normatauv7=Norm(row_tau_v7);

    //



    if ((normatauv7) > 0.5*sqrt(7)*(Idx_and_lim.CTL_for_TD(2)-Idx_and_lim.CTL_for_TD(1)))
    {
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
      return false;
    }

    MatrixXd<7,1> tau_h =tau_v7+tau_m7;

    tau = insertLimitTension7to8(tau_h, Idx_and_lim.indices_to_set(1),Idx_and_lim.limits_to_set(1));

    if ((tau.Min()>=Idx_and_lim.CTL_for_TD(1)) && (tau.Max()<=Idx_and_lim.CTL_for_TD(2)))
    {
      vars.tension_vector=arma::vec(tau.Data(),8,1,true);
      // vars.tension_vector= toArmaMat_generic(tau);
      return true;
    }
    else if ((tau.Min()<Idx_and_lim.CTL_for_TD(1)) && (tau.Max()>Idx_and_lim.CTL_for_TD(2)))
    {
      double maxdiff=tau.Max()-Idx_and_lim.CTL_for_TD(2);
      double mindiff=Idx_and_lim.CTL_for_TD(1)-tau.Min();

      if (maxdiff>mindiff)
      {
        Idx_and_lim.indices_to_set(2)=tau.MaxIdx();
        Idx_and_lim.limits_to_set(2)=Idx_and_lim.CTL_for_TD(2);
      } else
      {
        Idx_and_lim.indices_to_set(2)=tau.MinIdx();
        Idx_and_lim.limits_to_set(2)=Idx_and_lim.CTL_for_TD(1);
      }
    }

    else if (tau.Max()>Idx_and_lim.CTL_for_TD(2))
    {
      Idx_and_lim.indices_to_set(2)=tau.MaxIdx();
      Idx_and_lim.limits_to_set(2)=Idx_and_lim.CTL_for_TD(2);
    }
    else if (tau.Min()<Idx_and_lim.CTL_for_TD(1))
    {
      Idx_and_lim.indices_to_set(2)=tau.MinIdx();
      Idx_and_lim.limits_to_set(2)=Idx_and_lim.CTL_for_TD(1);
    }

//second try fixing non acceptable value to limit
    //adjust
    MatrixXd<6,1> tau_v6;
    MatrixXd<6,1> tau_m6;
    tau_m6.Fill(std::vector<double>{taumean,taumean,taumean,taumean,taumean,taumean});
    MatrixXd<6,6> smallestA;

    uint idxa =Idx_and_lim.indices_to_set.Max();
    uint idxb = Idx_and_lim.indices_to_set.Min();
    double limitA=Idx_and_lim.limits_to_set(Idx_and_lim.indices_to_set.MaxIdx());
    double limitB=Idx_and_lim.limits_to_set(Idx_and_lim.indices_to_set.MinIdx());

    counter=0;
    for (uint i=1;i<9;++i)
    {
      if (!(i==idxa) && (!(i==idxb)))
      {
        counter=counter+1;
        smallestA.SetCol(counter,A.GetCol(i));
      }
    }


    b=vars.platform.ext_load-smallestA*tau_m6-A.GetCol(idxa)*limitA-A.GetCol(idxb)*limitB;
    MatrixXd<6,6> smallestAT=smallestA.Transpose();
    MatrixXd<6,6> SMA_SMAT=smallestA*smallestAT;
   // MatrixXd<6,6> smallestMoorePenrose=smallestAT*Inverse(SMA_SMAT); //try catch block for inverse
    MatrixXd<6,6> smallestMoorePenrose;
    try {
      smallestMoorePenrose=smallestAT*Inverse(SMA_SMAT);
    }  catch (const std::runtime_error& e) {
      std::cout << "error:" << e.what() << std::endl;
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
      return false;
    }
    tau_v6=smallestMoorePenrose*b;

    MatrixXd<1,6> row_tau_v6=tau_v6.Transpose();
    double normatauv6=Norm(row_tau_v6);



    if (normatauv6 > 0.5*sqrt(6)*(Idx_and_lim.CTL_for_TD(2)-Idx_and_lim.CTL_for_TD(1)))
    {
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(1)+Idx_and_lim.CTL_for_TD(2));
      return false;
    }

    MatrixXd<6,1> tau_h1 =tau_v6+tau_m6;
    MatrixXd<7,1> tau_h2;


      tau_h2 = insertLimitTension6to7(tau_h1, idxb,limitB);
      tau = insertLimitTension7to8(tau_h2, idxa,limitA);


      if ((tau.Min()>=Idx_and_lim.CTL_for_TD(1)) && (tau.Max()<=Idx_and_lim.CTL_for_TD(2)))
     {
       vars.tension_vector=arma::vec(tau.Data(),8,1,true);
        //vars.tension_vector= toArmaMat_generic(tau);
       return true;
     }
     else
     {
       vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(1)+Idx_and_lim.CTL_for_TD(2));
       return false;
     }



}

MatrixXd<7,1> insertLimitTension6to7(const MatrixXd<6,1>& tau_h_,
                             const uint& index_to_set,
                             const double& TensionLimit)
{
    MatrixXd<7,1> tauout;
    for (uint i=1;i<8;++i)
    {
      if (i<index_to_set)
      {
        tauout(i,1)=tau_h_(i,1);
      } else if (i==index_to_set)
      {
        tauout(i,1)=TensionLimit;
      } else if (i>index_to_set)
      {
        tauout(i,1)=tau_h_(i-1,1);
      }
    }
      return tauout;
  }


MatrixXd<8,1> insertLimitTension7to8(const MatrixXd<7,1>& tau_h_,
                                  const uint& index_to_set,
                                  const double& TensionLimit)
{

    MatrixXd<8,1> tauout;
    for (uint i=1;i<9;++i)
    {
      if (i<index_to_set)
      {
        tauout(i,1)=tau_h_(i,1);
      } else if (i==index_to_set)
      {
        tauout(i,1)=TensionLimit;
      } else if (i>index_to_set)
      {
        tauout(i,1)=tau_h_(i-1,1);
      }
    }
      return tauout;

}


/**
arma::vec calcCableTensionDistribution(const arma::mat& geom_jacobian,
                                       const Vector6d& ext_load,
                                       const arma::vec& limits,
                                       const Index_and_limits& _Idx_and_lim)
{
  int tau_v_dimension=8;
  arma::mat extracted_cols=arma::zeros<arma::mat>(6,2);
  arma::mat A=geom_jacobian;

  if ((!(_Idx_and_lim.set_one_limit)) && (!(_Idx_and_lim.set_second_limit)))
   {
      tau_v_dimension=8;
      A=geom_jacobian;
   }
   else if ((_Idx_and_lim.set_one_limit) && (!(_Idx_and_lim.set_second_limit)))
   {
      tau_v_dimension=7;
      extracted_cols.col(0)=geom_jacobian.col(_Idx_and_lim.index_one_to_set);

      arma::vec mask=arma::zeros<arma::vec>(8);
      mask(_Idx_and_lim.index_one_to_set)=1;

      arma::uvec keepcol=arma::find(mask==0);
      A=geom_jacobian.cols(keepcol);
    }
    else if ((_Idx_and_lim.set_one_limit) && (_Idx_and_lim.set_second_limit))
    {
      tau_v_dimension=6;
      extracted_cols.col(0)=geom_jacobian.col(_Idx_and_lim.index_one_to_set);
      extracted_cols.col(1)=geom_jacobian.col(_Idx_and_lim.index_two_to_set);

      arma::vec mask=arma::zeros<arma::vec>(8);
      mask(_Idx_and_lim.index_one_to_set)=1;
      mask(_Idx_and_lim.index_two_to_set)=1;
      arma::uvec keepcol=arma::find(mask==0);
      A=geom_jacobian.cols(keepcol);
     }
     else
     {
     printColor('r', "ERROR: Cancel_index has unexpected values");
     }


  arma::vec tau_m = arma::ones<arma::vec>(tau_v_dimension)*0.5*(limits(0)+limits(1));
  arma::vec b=toArmaVec(ext_load)-extracted_cols*_Idx_and_lim.limits_to_set-A*tau_m;


  return arma::pinv(A)*b;

}
}
*/
bool GF::updateCablesTensionGF(RobotVars& vars)
  {

    MatrixXd<6,8> A =vars.geom_jacobian_l.Transpose();
    MatrixXd<6,2> Jc=A.GetCols<2>(2); //hard coded; tension controlled cables 2 and 3
    MatrixXd<6,6> Jd;
    Jd.SetCol(1,A.GetCol(1));
    Jd.SetBlock(1,2,A.GetCols<5>(4));
    MatrixXd<2,2> eye2;
    eye2.SetIdentity();
    N.SetBlock(1,1,-Inverse(Jd)*Jc);
    N.SetBlock(7,1,eye2);

   // external loads must be updated in cotroller
    MatrixXd<8,1> fp;
    fp.SetBlock(1,1,Inverse(Jd)*vars.platform.ext_load);
    fp(7,1)=0;
    fp(8,1)=0;
    MatrixXd<8,1> qmax=tau_max-fp;
    MatrixXd<8,1> qmin=tau_min-fp;
    MatrixXd<1,2> ni=N.GetRow(ii);
    MatrixXd<1,2> nj=N.GetRow(jj);
    MatrixXd<2,2> NIJ;
    NIJ.SetRow(1,ni);
    NIJ.SetRow(2,nj);
    MatrixXd<2,1> Q;
    Q.SetRow(1,qmin(ii));
    Q.SetRow(2,qmin(jj));
    MatrixXd<2,1> fc = Inverse(NIJ)*Q;
  //  Polygon=toArmaMat_generic(fc);
    Polygon =arma::mat(fc.Data(), 1, 2, true).t();

    MatrixXd<1, 8> In;
    In.SetZero();

    for (uint ij=1; ij<9 ; ij++)
    {
      MatrixXd<1, 1> first=N.GetRow(ij)*fc-qmin(ij);
      MatrixXd<1, 1> second=N.GetRow(ij)*fc-qmax(ij);

      if ( (std::abs(first(1,1))<eps2) ||  (std::abs(second(1,1))<eps2) || ((first(1,1)>0)&& (second(1,1)<0)) )
      {
        In(ij)=ij;
      } else
      {
        In(ij)=0;
      }
     }
    MatrixXd<2,1> vf=fc;

    int aa=1;
    int cnt=0;

    while (aa==1)
    {
      ni=N.GetRow(ii);
      nj=N.GetRow(jj);
      MatrixXd<1,2> ni_p1;
      ni_p1(1,1)=ni(2);
      ni_p1(1,2)=-ni(1);
      MatrixXd<1,2> ni_p2;
      ni_p2(1,1)=-ni(2);
      ni_p2(1,2)=ni(1);

      MatrixXd<1, 1> cond1=nj*fc-qmin(jj);
      MatrixXd<1, 1> cond2=nj*fc-qmax(jj);

      MatrixXd<1,2> ni_p;
      MatrixXd<1, 1> nj_nip1=nj*ni_p1.Transpose();
      if ((std::abs(cond1(1,1)))<eps2)
      {
        if ((nj_nip1(1,1))>=0)
        {
          ni_p=ni_p1;
        }
        else
        {
          ni_p=ni_p2;
        }
      }
      else if ((std::abs(cond2(1,1)))<eps2)
      {
        if ((nj_nip1(1,1))<=0)
        {
          ni_p=ni_p1;
        }
        else
        {
          ni_p=ni_p2;
        }
      }
      else
      {
        printColor('r', "Error1: no condition true ");
      }

      MatrixXd<1,8> alpha;

      alpha = calcalphas(fc, qmin, qmax, ni_p);
      MatrixXd<1,8> ralpha;
      ralpha= roundalphas(alpha,6);


      double al =alpha.Max();

      for (uint idx=1;idx<9;idx++)
      {
        if (ralpha(idx)>0)
        {
          if (alpha(idx)<al)
          {
            al=alpha(idx);
          }
        }
      }

      uint ll;

      if ( (al==alpha.Max()) && (ralpha(alpha.MaxIdx())<=0) )
      {
        printColor('r', "Error2: no acceptable alpha found ");
      }
      else
      {
        fc=fc+al*ni_p.Transpose();
        Polygon.insert_cols(Polygon.n_cols,arma::mat(fc.Data(), 1, 2, true).t());
        for (uint idx=1;idx<9;idx++)
        {
          if (alpha(idx)==al)
          {
           ll=idx;
          }
        }


        if (!(In(ll)==ll))
        {
          In(ll)=ll;
          vf=fc;
          Polygon =arma::mat(fc.Data(), 1, 2, true).t();
          jj=ii;
          ii=ll;
          aa=1;
        }
        else
        {

          if (Norm(fc-vf)>eps1)
          {
            jj=ii;
            ii=ll;
            aa=1;
          }
          else
          {
            if( (In(1)==1) &&
                (In(2)==2) &&
                (In(3)==3) &&
                (In(4)==4) &&
                (In(5)==5) &&
                (In(6)==6) &&
                (In(7)==7) &&
                (In(8)==8)  )
            {
              Polygon.shed_col(Polygon.n_cols-1);
              MatrixXd<2,1> tau_c= TDBaricenter(Polygon);

              MatrixXd<6,1> beq=vars.platform.ext_load-Jc*tau_c;
              MatrixXd<6,1> tau_prelim=Inverse(Jd)*beq;
              vars.tension_vector=arma::ones<arma::vec>(8)*(maxTension+minTension)/2;
              vars.tension_vector(0)=tau_prelim(1);
              vars.tension_vector(1)=tau_c(1);
              vars.tension_vector(2)=tau_c(2);
              vars.tension_vector(3)=tau_prelim(2);
              vars.tension_vector(4)=tau_prelim(3);
              vars.tension_vector(5)=tau_prelim(4);
              vars.tension_vector(6)=tau_prelim(5);
              vars.tension_vector(7)=tau_prelim(6);
              return true;
            }
            else
            {
              vars.tension_vector=arma::ones<arma::vec>(8)*(maxTension+minTension)/2;
              return false;
            }
          }
        }
      }

      cnt=cnt+1;
      if (cnt==1000)
      {aa=0;}
    }

    vars.tension_vector=arma::ones<arma::vec>(8)*(maxTension+minTension)/2;
    return false;
  };


  MatrixXd<1,8> GF::calcalphas(MatrixXd<2,1> fc_, MatrixXd<8,1> qmin_, MatrixXd<8,1> qmax_, MatrixXd<1,2> ni_p_)
  {
    MatrixXd<1,8> alpha;
    alpha.SetZero();
      for (int k=1; k<9;k++)
      {
        MatrixXd<1,2> nk =N.GetRow(k);
        MatrixXd<1, 1> nk_nip=nk*ni_p_.Transpose();
        MatrixXd<1, 1> cond1_=nk*fc_-qmin_(k);
        MatrixXd<1, 1> cond2_=nk*fc_-qmax_(k);


        if (nk_nip(1,1)>eps1)
        {
          if ( (cond1_(1,1)<0) && !(std::abs(cond1_(1,1))<eps2) )
          {
            alpha(k)=-cond1_(1,1)/nk_nip(1,1);
          }
          else if ( ( (cond1_(1,1)>0) && (cond2_(1,1)<0) ) || (std::abs(cond1_(1,1))<eps2)  )
          {
            alpha(k)=-cond2_(1,1)/nk_nip(1,1);
          }
          else
          {
            alpha(k)=1e16;
          }
        }
        else if ( ( nk_nip(1,1)<0 ) && (std::abs(nk_nip(1,1))>eps1) )
        {
          if ( (cond2_(1,1)>0) && !(std::abs(cond2_(1,1))<eps1) )
          {
            alpha(k)=-cond2_(1,1)/nk_nip(1,1);
          }
          else if ( ( (cond1_(1,1)>0) && (cond2_(1,1)<0) ) || (std::abs(cond2_(1,1))<eps1) )
          {
            alpha(k)=-cond1_(1,1)/nk_nip(1,1);
          }
          else
          {
            alpha(k)=1e16;
          }
        }
        else
        {
          if (k==ii)
          {
            alpha(k)=0;
          }
          else
          {
            alpha(k)=1e16;
          }
        }
      }
      return alpha;
  };


  MatrixXd<1,8> GF::roundalphas(MatrixXd<1,8> alpha_,int decimal)
  {
    double factor=std::pow(10.0,decimal);

    for (uint idx=1; idx<9; idx++)
    {
    alpha_(idx)=std::round(alpha_(idx)*factor)/factor;
    }
    return alpha_;
  };

  MatrixXd<2,1> GF::TDBaricenter(arma::mat Polygon_)
  {
    MatrixXd<2,1> tau_c;
    uint number_vertices_=Polygon_.n_cols;
    arma::rowvec wi(number_vertices_);
    for (uint i=0; i<number_vertices_;i++)
    {
      if (i==0)
      {
        double norm1= arma::norm(Polygon_.col(i)-Polygon_.col(number_vertices_-1));
        double norm2= arma::norm(Polygon_.col(i)-Polygon_.col(i+1));

        if (norm(Polygon_.col(i))==0)
        {
          wi(i)=0;
        }
         else
        {
           wi(i)=(norm1+norm2)/norm(Polygon_.col(i));
        }
      }
      else if (i==number_vertices_-1)
      {
        double norm1= arma::norm(Polygon_.col(i)-Polygon_.col(number_vertices_-2));
        double norm2= arma::norm(Polygon_.col(i)-Polygon_.col(0));
        if (norm(Polygon_.col(i))==0)
        {
          wi(i)=0;
        }
        else
        {
          wi(i)=(norm1+norm2)/norm(Polygon_.col(i));
        }
      }
      else
      {
        double norm1= arma::norm(Polygon_.col(i)-Polygon_.col(i-1));
        double norm2= arma::norm(Polygon_.col(i)-Polygon_.col(i+1));
        if (norm(Polygon_.col(i))==0)
        {
          wi(i)=0;
        }
        else
        {
          wi(i)=(norm1+norm2)/norm(Polygon_.col(i));
        }
      }
     }

    arma::vec nom ={0,0};
     double denom=0;
    for (uint i=0; i<number_vertices_;i++)
    {
      nom=nom+wi(i)*Polygon_.col(i);
      denom=denom+wi(i);
    }

        arma::vec lambda=nom/denom;
    tau_c(1)=lambda(0);
    tau_c(2)=lambda(1);
    return tau_c;
  };





}

