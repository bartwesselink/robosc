import { ComponentFixture, TestBed } from '@angular/core/testing';

import { OverviewIdeaComponent } from './overview-idea.component';

describe('IntroductionIdeaComponent', () => {
  let component: OverviewIdeaComponent;
  let fixture: ComponentFixture<OverviewIdeaComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ OverviewIdeaComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(OverviewIdeaComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
